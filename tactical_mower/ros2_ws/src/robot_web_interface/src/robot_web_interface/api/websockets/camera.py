"""Camera WebSocket handlers"""
import asyncio
import base64
import cv2
import logging
from concurrent.futures import ThreadPoolExecutor
from fastapi import WebSocket, WebSocketDisconnect
from ...ros_interface.robot_node import RobotNode

# Import websockets exceptions if available (used by uvicorn)
try:
    from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
except ImportError:
    # Fallback if websockets library not directly available
    ConnectionClosedOK = type('ConnectionClosedOK', (Exception,), {})
    ConnectionClosedError = type('ConnectionClosedError', (Exception,), {})

# Thread pool for CPU-intensive image processing (non-blocking)
# Increased to 6 workers to handle 30 FPS streams efficiently
_image_executor = ThreadPoolExecutor(max_workers=6, thread_name_prefix="img_proc")

# Maximum resolution for web streams (reduces bandwidth and processing load)
WEB_STREAM_MAX_WIDTH = 1280
WEB_STREAM_MAX_HEIGHT = 720
# Smaller resolution for thermal cameras (they don't need high detail for web streaming)
THERMAL_STREAM_MAX_WIDTH = 640
THERMAL_STREAM_MAX_HEIGHT = 480

# Logger for frame resolution verification
logger = logging.getLogger(__name__)

# Track if we've logged resolution info for each camera (to avoid spam)
_resolution_logged = {
    'main': False,
    'thermal1': False,
    'thermal2': False,
    'rgb2': False,
    'lidar_debug': False
}


def _process_frame_to_jpeg_bytes(frame, camera_type='unknown', max_width=WEB_STREAM_MAX_WIDTH, 
                                  max_height=WEB_STREAM_MAX_HEIGHT, jpeg_quality=70):
    """Process frame to raw JPEG bytes (no Base64) - much faster and smaller
    Returns raw bytes that can be sent directly via WebSocket binary"""
    if frame is None:
        return None
    
    height, width = frame.shape[:2]
    frame_to_encode = frame
    
    # Resize only if needed
    if width > max_width or height > max_height:
        scale = min(max_width / width, max_height / height)
        new_width = int(width * scale)
        new_height = int(height * scale)
        
        if not _resolution_logged.get(camera_type, False):
            logger.info(f'[{camera_type}] Scaling: {width}x{height} -> {new_width}x{new_height}')
            _resolution_logged[camera_type] = True
        
        # INTER_NEAREST is fastest
        frame_to_encode = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
    else:
        if not _resolution_logged.get(camera_type, False):
            logger.info(f'[{camera_type}] Frame {width}x{height} - no scaling')
            _resolution_logged[camera_type] = True
    
    # Encode to JPEG - minimal params for speed
    _, buffer = cv2.imencode('.jpg', frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    return buffer.tobytes()


def _process_frame_sync(frame, camera_type='unknown', max_width=WEB_STREAM_MAX_WIDTH, 
                        max_height=WEB_STREAM_MAX_HEIGHT, jpeg_quality=70):
    """Process frame to Base64 string (legacy, for JSON transport)"""
    jpeg_bytes = _process_frame_to_jpeg_bytes(frame, camera_type, max_width, max_height, jpeg_quality)
    if jpeg_bytes is None:
        return None
    return base64.b64encode(jpeg_bytes).decode('utf-8')


async def _process_frame_async(frame, camera_type='unknown', max_width=WEB_STREAM_MAX_WIDTH, 
                               max_height=WEB_STREAM_MAX_HEIGHT, jpeg_quality=70):
    """Process frame to Base64 string asynchronously (legacy)"""
    if frame is None:
        return None
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(
        _image_executor, _process_frame_sync, frame, camera_type, max_width, max_height, jpeg_quality
    )


async def _process_frame_to_bytes_async(frame, camera_type='unknown', max_width=WEB_STREAM_MAX_WIDTH, 
                                        max_height=WEB_STREAM_MAX_HEIGHT, jpeg_quality=70):
    """Process frame to raw JPEG bytes asynchronously - FASTEST option
    Use with websocket.send_bytes() for minimal latency"""
    if frame is None:
        return None
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(
        _image_executor, _process_frame_to_jpeg_bytes, frame, camera_type, max_width, max_height, jpeg_quality
    )


async def websocket_camera_main(websocket: WebSocket, ros_node: RobotNode):
    """Main camera WebSocket stream - optimized for low latency, non-blocking
    FIX: Removed re-check after processing that caused frames to be skipped indefinitely"""
    await websocket.accept()
    camera_type = 'main'
    # Quality: default LQ (CPU/bandwidth saver). Fullscreen UI may request ?quality=high.
    _hq = websocket.query_params.get('quality') == 'high'
    _max_w = WEB_STREAM_MAX_WIDTH if _hq else THERMAL_STREAM_MAX_WIDTH
    _max_h = WEB_STREAM_MAX_HEIGHT if _hq else THERMAL_STREAM_MAX_HEIGHT
    _jq = 70 if _hq else 35
    # Register this stream as active and add this connection
    ros_node.connection_manager.register_camera_stream(camera_type)
    ros_node.connection_manager.add_camera_connection(camera_type, websocket)
    
    last_sent_frame_time = 0.0
    last_sent_frame_timestamp = 0.0  # Track timestamp of last sent frame to skip old frames
    min_frame_interval = 0.033  # ~30 FPS max (33ms between frames)
    
    # NO BLOCKING WAIT - start immediately, streams should never block communication
    try:
        while True:
            # Get frame and timestamp (non-blocking, always gets latest)
            frame, frame_timestamp = ros_node.get_frame()
            
            # Only send if frame is newer than last sent (newest-first)
            if frame is not None and frame_timestamp > last_sent_frame_timestamp:
                # Throttle sending to avoid overwhelming the client (max ~30 FPS)
                current_time = asyncio.get_event_loop().time()
                time_since_last = current_time - last_sent_frame_time
                if time_since_last < min_frame_interval:
                    # Sleep for the remaining time to avoid busy waiting
                    sleep_time = min_frame_interval - time_since_last
                    await asyncio.sleep(sleep_time)
                    continue
                
                try:
                    # Process frame asynchronously (non-blocking - runs in thread pool)
                    jpeg_bytes = await _process_frame_to_bytes_async(
                        frame,
                        camera_type='main',
                        max_width=_max_w,
                        max_height=_max_h,
                        jpeg_quality=_jq
                    )
                    if jpeg_bytes:
                        await websocket.send_bytes(jpeg_bytes)
                        last_sent_frame_time = asyncio.get_event_loop().time()
                        last_sent_frame_timestamp = frame_timestamp
                except (ConnectionClosedOK, ConnectionClosedError, WebSocketDisconnect):
                    # Connection closed - this is normal, break the loop
                    raise  # Re-raise to exit the outer loop
                except Exception as e:
                    # Log other errors but continue - don't block
                    logger.warning(f'Frame processing/sending error (non-blocking): {e}', exc_info=True)
            else:
                # No new frame available yet, wait a bit before checking again
                await asyncio.sleep(0.005)  # main: fast pickup for manual control
    except (WebSocketDisconnect, ConnectionClosedOK, ConnectionClosedError):
        # Normal WebSocket close - not an error
        logger.debug(f'Camera WebSocket disconnected normally: {camera_type}')
    except Exception as e:
        logger.error(f'Camera WebSocket error for {camera_type}: {e}', exc_info=True)
    finally:
        # Remove this connection and unregister stream only if no connections remain
        ros_node.connection_manager.remove_camera_connection(camera_type, websocket)
        # Only unregister if no clients are connected to this stream
        if not ros_node.connection_manager.has_camera_connections(camera_type):
            ros_node.connection_manager.unregister_camera_stream(camera_type)


async def websocket_camera_thermal1(websocket: WebSocket, ros_node: RobotNode):
    """Thermal camera 1 WebSocket stream - BINARY mode for lowest latency
    Sends raw JPEG bytes directly - 33% smaller than Base64, much faster"""
    await websocket.accept()
    camera_type = 'thermal1'
    ros_node.connection_manager.register_camera_stream(camera_type)
    ros_node.connection_manager.add_camera_connection(camera_type, websocket)
    
    last_sent_frame_timestamp = 0.0
    
    try:
        while True:
            frame, frame_timestamp = ros_node.get_thermal1_frame()
            
            if frame is not None and frame_timestamp > last_sent_frame_timestamp:
                try:
                    # Process to raw JPEG bytes (no Base64 = faster + smaller)
                    jpeg_bytes = await _process_frame_to_bytes_async(
                        frame,
                        camera_type='thermal1',
                        max_width=THERMAL_STREAM_MAX_WIDTH,
                        max_height=THERMAL_STREAM_MAX_HEIGHT,
                        jpeg_quality=35  # Lower quality for speed
                    )
                    
                    if jpeg_bytes:
                        # Send as binary - much faster than JSON+Base64
                        await websocket.send_bytes(jpeg_bytes)
                        last_sent_frame_timestamp = frame_timestamp
                except (ConnectionClosedOK, ConnectionClosedError, WebSocketDisconnect):
                    raise
                except Exception as e:
                    logger.warning(f'Frame error: {e}')
            else:
                await asyncio.sleep(0.05)  # 20Hz polling when no new frame
    except (WebSocketDisconnect, ConnectionClosedOK, ConnectionClosedError):
        logger.debug(f'Camera WebSocket closed: {camera_type}')
    except Exception as e:
        logger.error(f'Camera WebSocket error {camera_type}: {e}', exc_info=True)
    finally:
        ros_node.connection_manager.remove_camera_connection(camera_type, websocket)
        if not ros_node.connection_manager.has_camera_connections(camera_type):
            ros_node.connection_manager.unregister_camera_stream(camera_type)


async def websocket_camera_thermal2(websocket: WebSocket, ros_node: RobotNode):
    """Thermal camera 2 WebSocket stream - BINARY mode for lowest latency"""
    await websocket.accept()
    camera_type = 'thermal2'
    # Quality: default LQ (CPU/bandwidth saver). Fullscreen UI may request ?quality=high.
    _hq = websocket.query_params.get('quality') == 'high'
    _max_w = WEB_STREAM_MAX_WIDTH if _hq else THERMAL_STREAM_MAX_WIDTH
    _max_h = WEB_STREAM_MAX_HEIGHT if _hq else THERMAL_STREAM_MAX_HEIGHT
    _jq = 70 if _hq else 35
    ros_node.connection_manager.register_camera_stream(camera_type)
    ros_node.connection_manager.add_camera_connection(camera_type, websocket)

    last_sent_frame_timestamp = 0.0

    try:
        while True:
            frame, frame_timestamp = ros_node.get_thermal2_frame()

            if frame is not None and frame_timestamp > last_sent_frame_timestamp:
                try:
                    jpeg_bytes = await _process_frame_to_bytes_async(
                        frame, 'thermal2', _max_w, _max_h, _jq
                    )
                    if jpeg_bytes:
                        await websocket.send_bytes(jpeg_bytes)
                        last_sent_frame_timestamp = frame_timestamp
                except (ConnectionClosedOK, ConnectionClosedError, WebSocketDisconnect):
                    raise
                except Exception as e:
                    logger.warning(f'Frame error: {e}')
            else:
                await asyncio.sleep(0.05)
    except (WebSocketDisconnect, ConnectionClosedOK, ConnectionClosedError):
        logger.debug(f'Camera WebSocket closed: {camera_type}')
    except Exception as e:
        logger.error(f'Camera error {camera_type}: {e}', exc_info=True)
    finally:
        ros_node.connection_manager.remove_camera_connection(camera_type, websocket)
        if not ros_node.connection_manager.has_camera_connections(camera_type):
            ros_node.connection_manager.unregister_camera_stream(camera_type)


async def websocket_camera_rgb2(websocket: WebSocket, ros_node: RobotNode):
    """RGB2 camera WebSocket stream (Axis channel 1) - BINARY mode for lowest latency"""
    await websocket.accept()
    camera_type = 'rgb2'
    # Quality: default LQ (CPU/bandwidth saver). Fullscreen UI may request ?quality=high.
    _hq = websocket.query_params.get('quality') == 'high'
    _max_w = WEB_STREAM_MAX_WIDTH if _hq else THERMAL_STREAM_MAX_WIDTH
    _max_h = WEB_STREAM_MAX_HEIGHT if _hq else THERMAL_STREAM_MAX_HEIGHT
    _jq = 70 if _hq else 35
    ros_node.connection_manager.register_camera_stream(camera_type)
    ros_node.connection_manager.add_camera_connection(camera_type, websocket)

    last_sent_frame_timestamp = 0.0

    try:
        while True:
            frame, frame_timestamp = ros_node.get_rgb2_frame()

            if frame is not None and frame_timestamp > last_sent_frame_timestamp:
                try:
                    jpeg_bytes = await _process_frame_to_bytes_async(
                        frame, 'rgb2', _max_w, _max_h, _jq
                    )
                    if jpeg_bytes:
                        await websocket.send_bytes(jpeg_bytes)
                        last_sent_frame_timestamp = frame_timestamp
                except (ConnectionClosedOK, ConnectionClosedError, WebSocketDisconnect):
                    raise
                except Exception as e:
                    logger.warning(f'Frame error: {e}')
            else:
                await asyncio.sleep(0.05)
    except (WebSocketDisconnect, ConnectionClosedOK, ConnectionClosedError):
        logger.debug(f'Camera WebSocket closed: {camera_type}')
    except Exception as e:
        logger.error(f'Camera error {camera_type}: {e}', exc_info=True)
    finally:
        ros_node.connection_manager.remove_camera_connection(camera_type, websocket)
        if not ros_node.connection_manager.has_camera_connections(camera_type):
            ros_node.connection_manager.unregister_camera_stream(camera_type)


async def websocket_camera_lidar_debug(websocket: WebSocket, ros_node: RobotNode):
    """LIDAR debug camera WebSocket stream (Livox obstacle image) - optimized for low latency, non-blocking
    FIX: Removed re-check after processing that caused frames to be skipped indefinitely"""
    await websocket.accept()
    camera_type = 'lidar_debug'
    # Register this stream as active and add this connection
    ros_node.connection_manager.register_camera_stream(camera_type)
    ros_node.connection_manager.add_camera_connection(camera_type, websocket)
    
    last_sent_frame_time = 0.0
    last_sent_frame_timestamp = 0.0  # Track timestamp of last sent frame to skip old frames
    min_frame_interval = 0.033  # ~30 FPS max (33ms between frames)
    
    try:
        while True:
            # Get frame and timestamp (non-blocking, always gets latest)
            frame, frame_timestamp = ros_node.get_lidar_debug_frame()
            
            # Only send if frame is newer than last sent (newest-first)
            if frame is not None and frame_timestamp > last_sent_frame_timestamp:
                # Throttle sending to avoid overwhelming the client (max ~30 FPS)
                current_time = asyncio.get_event_loop().time()
                time_since_last = current_time - last_sent_frame_time
                if time_since_last < min_frame_interval:
                    # Sleep for the remaining time to avoid busy waiting
                    sleep_time = min_frame_interval - time_since_last
                    await asyncio.sleep(sleep_time)
                    continue
                
                try:
                    # Process frame asynchronously (non-blocking - runs in thread pool)
                    jpg_base64 = await _process_frame_async(
                        frame,
                        camera_type='lidar_debug',
                        max_width=WEB_STREAM_MAX_WIDTH,
                        max_height=WEB_STREAM_MAX_HEIGHT,
                        jpeg_quality=70
                    )
                    # SEND the processed frame immediately - do NOT skip it
                    if jpg_base64:
                        await websocket.send_json({
                            "type": "camera",
                            "data": jpg_base64
                        })
                        last_sent_frame_time = asyncio.get_event_loop().time()
                        last_sent_frame_timestamp = frame_timestamp
                except (ConnectionClosedOK, ConnectionClosedError, WebSocketDisconnect):
                    # Connection closed - this is normal, break the loop
                    raise  # Re-raise to exit the outer loop
                except Exception as e:
                    # Log other errors but continue - don't block
                    logger.warning(f'Frame processing/sending error (non-blocking): {e}', exc_info=True)
            else:
                # No new frame available yet, wait a bit before checking again
                await asyncio.sleep(0.033)  # Check at ~30Hz when no frames
    except (WebSocketDisconnect, ConnectionClosedOK, ConnectionClosedError):
        # Normal WebSocket close - not an error
        logger.debug(f'Camera WebSocket disconnected normally: {camera_type}')
    except Exception as e:
        logger.error(f'Camera WebSocket error for {camera_type}: {e}', exc_info=True)
    finally:
        # Remove this connection and unregister stream only if no connections remain
        ros_node.connection_manager.remove_camera_connection(camera_type, websocket)
        # Only clear frame cache if no clients are connected to this stream
        if not ros_node.connection_manager.has_camera_connections(camera_type):
            ros_node.connection_manager.unregister_camera_stream(camera_type)
            ros_node.status_manager.clear_frame("lidar_debug")



async def websocket_camera_depth_debug(websocket: WebSocket, ros_node: RobotNode):
    """Depth debug camera WebSocket stream (Livox obstacle image) - optimized for low latency, non-blocking
    FIX: Removed re-check after processing that caused frames to be skipped indefinitely"""
    await websocket.accept()
    camera_type = 'depth_debug'
    # Register this stream as active and add this connection
    ros_node.connection_manager.register_camera_stream(camera_type)
    ros_node.connection_manager.add_camera_connection(camera_type, websocket)
    
    last_sent_frame_time = 0.0
    last_sent_frame_timestamp = 0.0  # Track timestamp of last sent frame to skip old frames
    min_frame_interval = 0.033  # ~30 FPS max (33ms between frames)
    
    try:
        while True:
            # Get frame and timestamp (non-blocking, always gets latest)
            frame, frame_timestamp = ros_node.get_depth_debug_frame()
            
            # Only send if frame is newer than last sent (newest-first)
            if frame is not None and frame_timestamp > last_sent_frame_timestamp:
                # Throttle sending to avoid overwhelming the client (max ~30 FPS)
                current_time = asyncio.get_event_loop().time()
                time_since_last = current_time - last_sent_frame_time
                if time_since_last < min_frame_interval:
                    # Sleep for the remaining time to avoid busy waiting
                    sleep_time = min_frame_interval - time_since_last
                    await asyncio.sleep(sleep_time)
                    continue
                
                try:
                    # Process frame asynchronously (non-blocking - runs in thread pool)
                    jpg_base64 = await _process_frame_async(
                        frame,
                        camera_type='depth_debug',
                        max_width=WEB_STREAM_MAX_WIDTH,
                        max_height=WEB_STREAM_MAX_HEIGHT,
                        jpeg_quality=70
                    )
                    # SEND the processed frame immediately - do NOT skip it
                    if jpg_base64:
                        await websocket.send_json({
                            "type": "camera",
                            "data": jpg_base64
                        })
                        last_sent_frame_time = asyncio.get_event_loop().time()
                        last_sent_frame_timestamp = frame_timestamp
                except (ConnectionClosedOK, ConnectionClosedError, WebSocketDisconnect):
                    # Connection closed - this is normal, break the loop
                    raise  # Re-raise to exit the outer loop
                except Exception as e:
                    # Log other errors but continue - don't block
                    logger.warning(f'Frame processing/sending error (non-blocking): {e}', exc_info=True)
            else:
                # No new frame available yet, wait a bit before checking again
                await asyncio.sleep(0.033)  # Check at ~30Hz when no frames
    except (WebSocketDisconnect, ConnectionClosedOK, ConnectionClosedError):
        # Normal WebSocket close - not an error
        logger.debug(f'Camera WebSocket disconnected normally: {camera_type}')
    except Exception as e:
        logger.error(f'Camera WebSocket error for {camera_type}: {e}', exc_info=True)
    finally:
        # Remove this connection and unregister stream only if no connections remain
        ros_node.connection_manager.remove_camera_connection(camera_type, websocket)
        # Only clear frame cache if no clients are connected to this stream
        if not ros_node.connection_manager.has_camera_connections(camera_type):
            ros_node.connection_manager.unregister_camera_stream(camera_type)
            ros_node.status_manager.clear_frame("depth_debug")



async def websocket_camera_person_detection(websocket: WebSocket, ros_node: RobotNode):
    """Person detection WebSocket - reads MJPEG from live_detect.py port 8080"""
    await websocket.accept()
    camera_type = 'person_detection'
    # Quality: default LQ (forward every 2nd frame to halve bandwidth). HQ = passthrough.
    _hq = websocket.query_params.get('quality') == 'high'
    _frame_skip = 1 if _hq else 2
    _pd_counter = 0
    ros_node.connection_manager.register_camera_stream(camera_type)
    ros_node.connection_manager.add_camera_connection(camera_type, websocket)

    import urllib.request
    import queue
    import threading

    frame_queue = queue.Queue(maxsize=2)
    BOUNDARY = b'--frame'
    JPEG_START = b'\xff\xd8'
    # MJPEG_LEAK_FIX_v1
    stop_event = threading.Event()
    stream_holder = {}  # mutable container so finally can close it

    def mjpeg_reader():
        try:
            stream = urllib.request.urlopen('http://192.168.10.169:8080/stream', timeout=10)
            stream_holder['s'] = stream
            buf = b''
            while not stop_event.is_set():
                buf += stream.read(8192)
                while True:
                    # Find two consecutive boundaries to extract one frame
                    b1 = buf.find(BOUNDARY)
                    if b1 == -1:
                        break
                    b2 = buf.find(BOUNDARY, b1 + len(BOUNDARY))
                    if b2 == -1:
                        break
                    chunk = buf[b1:b2]
                    buf = buf[b2:]
                    # Find JPEG start within chunk
                    js = chunk.find(JPEG_START)
                    if js == -1:
                        continue
                    frame = chunk[js:]
                    if len(frame) < 100:
                        continue
                    try:
                        frame_queue.put_nowait(frame)
                    except queue.Full:
                        try:
                            frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                        frame_queue.put_nowait(frame)
        except Exception as e:
            logger.error(f'MJPEG reader error: {e}')
        finally:
            try:
                s = stream_holder.get('s')
                if s is not None:
                    s.close()
            except Exception:
                pass
            frame_queue.put(None)

    import threading
    threading.Thread(target=mjpeg_reader, daemon=True).start()

    try:
        loop = asyncio.get_event_loop()
        while True:
            jpeg_bytes = await loop.run_in_executor(_image_executor, frame_queue.get, True, 5.0)
            if jpeg_bytes is None:
                break
            _pd_counter += 1
            if _frame_skip > 1 and (_pd_counter % _frame_skip) != 0:
                continue
            if _hq:
                # HQ: passthrough native MJPEG (full resolution, full quality)
                await websocket.send_bytes(jpeg_bytes)
            else:
                # LQ: decode + downscale + re-encode at low quality for visible SD feel
                try:
                    import numpy as np
                    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                    frame_dec = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame_dec is not None:
                        # Force a real downscale — source from jetson-ai is 640x360,
                        # so the THERMAL_STREAM_MAX (640x480) cap was a no-op. Use 320 wide
                        # + q=25 to make LQ visibly soft vs HQ passthrough.
                        re_jpeg = await _process_frame_to_bytes_async(
                            frame_dec, "person_detection",
                            320, 240, 25
                        )
                        if re_jpeg:
                            await websocket.send_bytes(re_jpeg)
                            continue
                    await websocket.send_bytes(jpeg_bytes)
                except Exception:
                    await websocket.send_bytes(jpeg_bytes)
    except (WebSocketDisconnect, ConnectionClosedOK, ConnectionClosedError):
        logger.debug('Person detection WebSocket closed')
    except Exception as e:
        logger.error(f'Person detection error: {e}', exc_info=True)
    finally:
        # MJPEG_LEAK_FIX_v1 signal mjpeg_reader thread to exit + close urllib stream
        try:
            stop_event.set()
        except Exception:
            pass
        try:
            s = stream_holder.get('s')
            if s is not None:
                s.close()
        except Exception:
            pass
        ros_node.connection_manager.remove_camera_connection(camera_type, websocket)
        if not ros_node.connection_manager.has_camera_connections(camera_type):
            ros_node.connection_manager.unregister_camera_stream(camera_type)
