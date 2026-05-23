        let allEvents = [];
        let selectedEventId = null;
        let selectedDate = null; // YYYY-MM-DD
        let datesWithEvents = new Set();
        let flatpickrInstance = null;
        let editMode = false;
        let selectedEventIds = new Set();
        let downloadCancelled = false;
        let downloadProgress = {
            current: 0,
            total: 0,
            currentFile: ''
        };
        
        // Pagination state
        const PAGE_SIZE = 50;
        let currentOffset = 0;
        let totalEvents = 0;
        let isLoadingMore = false;

        function parseEventDateTime(isoStr) {
            if (!isoStr) return null;
            const d = new Date(isoStr);
            if (isNaN(d.getTime())) return null;
            return d;
        }

        function dateKeyFromIso(isoStr) {
            const d = parseEventDateTime(isoStr);
            if (!d) return null;
            const pad = (n) => String(n).padStart(2, '0');
            return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}`;
        }

        function formatDateTimeLocal(isoStr) {
            const d = parseEventDateTime(isoStr);
            if (!d) return isoStr || '';
            const pad = (n) => String(n).padStart(2, '0');
            const year = d.getFullYear();
            const month = pad(d.getMonth() + 1);
            const day = pad(d.getDate());
            const hour = pad(d.getHours());
            const minute = pad(d.getMinutes());
            return `${year}-${month}-${day} ${hour}:${minute}`;
        }

        function updateDateUi() {
            const labelEl = document.getElementById('selected-date-label');

            if (!selectedDate) {
                labelEl.textContent = '–';
                if (flatpickrInstance) {
                    flatpickrInstance.clear();
                }
                return;
            }

            labelEl.textContent = selectedDate;
            if (flatpickrInstance) {
                flatpickrInstance.setDate(selectedDate, false);
            }
            updateCalendarEventIndicators();
        }

        function updateCalendarEventIndicators() {
            if (!flatpickrInstance || !flatpickrInstance.calendarContainer) return;
            
            // Remove existing event indicators
            flatpickrInstance.calendarContainer.querySelectorAll('.flatpickr-day.has-event').forEach(day => {
                day.classList.remove('has-event');
            });

            // Add event indicators to days with events
            const allDays = flatpickrInstance.calendarContainer.querySelectorAll('.flatpickr-day:not(.flatpickr-disabled)');
            allDays.forEach(day => {
                if (day.dateObj) {
                    const dayDate = new Date(day.dateObj);
                    const pad = (n) => String(n).padStart(2, '0');
                    const dayKey = `${dayDate.getFullYear()}-${pad(dayDate.getMonth() + 1)}-${pad(dayDate.getDate())}`;
                    if (datesWithEvents.has(dayKey)) {
                        day.classList.add('has-event');
                    }
                }
            });
        }

        async function loadEvents(reset = true) {
            if (isLoadingMore) return;
            
            try {
                isLoadingMore = true;
                
                if (reset) {
                    currentOffset = 0;
                    allEvents = [];
                }
                
                const resp = await fetch(`/api/events?limit=${PAGE_SIZE}&offset=${currentOffset}`);
                const data = await resp.json();
                const newEvents = data.events || [];
                totalEvents = data.total || 0;
                
                // Add new events to existing list
                allEvents = allEvents.concat(newEvents);
                currentOffset = allEvents.length;

                // Update datesWithEvents set with all loaded events
                newEvents.forEach(ev => {
                    const key = dateKeyFromIso(ev.event_time);
                    if (key) {
                        datesWithEvents.add(key);
                    }
                });

                // Initialize or validate selectedDate (only on first load)
                if (reset) {
                    if (!selectedDate || !datesWithEvents.has(selectedDate)) {
                        const todayKey = dateKeyFromIso(new Date().toISOString());
                        if (todayKey && datesWithEvents.has(todayKey)) {
                            selectedDate = todayKey;
                        } else if (allEvents.length > 0) {
                            const key = dateKeyFromIso(allEvents[0].event_time);
                            selectedDate = key;
                        } else {
                            selectedDate = todayKey;
                        }
                    }
                }

                updateDateUi();
                renderEventList();
                updateLoadMoreButton();
                
                // Update calendar indicators after events are loaded
                if (flatpickrInstance) {
                    updateCalendarEventIndicators();
                }
            } catch (e) {
                console.error('Failed to load events', e);
            } finally {
                isLoadingMore = false;
            }
        }

        async function loadMoreEvents() {
            await loadEvents(false);
        }

        function updateLoadMoreButton() {
            let loadMoreBtn = document.getElementById('load-more-events-btn');
            const listEl = document.getElementById('event-list');
            
            if (allEvents.length >= totalEvents) {
                // All events loaded, remove button if exists
                if (loadMoreBtn) {
                    loadMoreBtn.remove();
                }
                return;
            }
            
            // Create or update button
            if (!loadMoreBtn) {
                loadMoreBtn = document.createElement('button');
                loadMoreBtn.id = 'load-more-events-btn';
                loadMoreBtn.className = 'load-more-btn';
                loadMoreBtn.textContent = 'Mehr laden';
                loadMoreBtn.onclick = loadMoreEvents;
                loadMoreBtn.style.cssText = `
                    width: 100%;
                    padding: 12px;
                    margin-top: 10px;
                    background: var(--brand-blue);
                    color: white;
                    border: none;
                    border-radius: 4px;
                    cursor: pointer;
                    font-size: 1em;
                    transition: background 0.3s;
                `;
                loadMoreBtn.onmouseover = () => loadMoreBtn.style.background = '#2a42ff';
                loadMoreBtn.onmouseout = () => loadMoreBtn.style.background = 'var(--brand-blue)';
                listEl.appendChild(loadMoreBtn);
            }
            
            // Update button text
            const remaining = totalEvents - allEvents.length;
            loadMoreBtn.textContent = `Mehr laden (${remaining} verbleibend)`;
            loadMoreBtn.disabled = isLoadingMore;
        }

        function renderEventList() {
            const listEl = document.getElementById('event-list');
            listEl.innerHTML = '';

            const eventsForDay = allEvents.filter(ev => {
                const key = dateKeyFromIso(ev.event_time);
                return !selectedDate || key === selectedDate;
            });

            if (!eventsForDay.length) {
                const empty = document.createElement('div');
                empty.style.color = '#888';
                empty.style.fontSize = '0.9em';
                empty.textContent = 'Keine Sicherheitsereignisse für diesen Tag.';
                listEl.appendChild(empty);
                return;
            }

            eventsForDay.forEach(event => {
                const item = document.createElement('div');
                item.className = 'event-item event-card';  // UI sprint #7: card layout
                if (editMode) item.classList.add('edit-mode');
                if (event.event_id === selectedEventId && !editMode) item.classList.add('active');

                // Checkbox for edit mode (kept — operator still needs bulk-select)
                if (editMode) {
                    const checkboxWrapper = document.createElement('div');
                    checkboxWrapper.className = 'event-checkbox-wrapper';
                    const checkbox = document.createElement('input');
                    checkbox.type = 'checkbox';
                    checkbox.className = 'event-checkbox';
                    checkbox.checked = selectedEventIds.has(event.event_id);
                    checkbox.addEventListener('change', (e) => {
                        e.stopPropagation();
                        if (checkbox.checked) selectedEventIds.add(event.event_id);
                        else selectedEventIds.delete(event.event_id);
                        updateSelectedCount();
                    });
                    checkboxWrapper.appendChild(checkbox);
                    item.appendChild(checkboxWrapper);
                }

                // UI sprint #7: 16:9 thumbnail of the snapshot frame.
                // Falls back to a German label when no frame.jpg exists
                // (legacy events from before the arrival pipeline).
                const thumbWrap = document.createElement('div');
                thumbWrap.className = 'event-thumb';
                const thumb = document.createElement('img');
                thumb.alt = 'Vorschau';
                thumb.loading = 'lazy';
                thumb.src = `/api/events/${event.event_id}/frame`;
                thumb.onerror = () => {
                    thumb.style.display = 'none';
                    const placeholder = document.createElement('div');
                    placeholder.className = 'event-thumb-placeholder';
                    placeholder.textContent = 'Kein Vorschaubild';
                    thumbWrap.appendChild(placeholder);
                };
                thumbWrap.appendChild(thumb);

                // Plain-German class label with emoji.
                // Class info lives in metadata.json, not the index, so we map
                // by event_type which IS in the index. arrival = generic 🔔.
                const evType = (event.event_type || '').toLowerCase();
                const emojiMap = {
                    'persondetect': '🚶', 'person': '🚶',
                    'firedetect': '🔥', 'fire': '🔥',
                    'arrival': '🔔',
                };
                const labelMap = {
                    'persondetect': 'Person', 'person': 'Person',
                    'firedetect': 'Feuer', 'fire': 'Feuer',
                    'arrival': 'Ankunft',
                };
                const emoji = emojiMap[evType] || '🔔';
                const labelText = labelMap[evType] || (event.event_type || 'Ereignis');

                const labelEl = document.createElement('div');
                labelEl.className = 'event-card-label';
                labelEl.innerHTML = `<span class="event-card-emoji">${emoji}</span> <span>${labelText}</span>`;

                const timeEl = document.createElement('div');
                timeEl.className = 'event-card-time';
                timeEl.textContent = formatDateTimeLocal(event.event_time);

                const flags = document.createElement('div');
                flags.className = 'event-flags';
                if (!event.has_videos) {
                    const noVid = document.createElement('span');
                    noVid.className = 'flag warn';
                    noVid.textContent = 'Kein Video';
                    flags.appendChild(noVid);
                }

                item.appendChild(thumbWrap);
                item.appendChild(labelEl);
                item.appendChild(timeEl);
                if (flags.childElementCount > 0) item.appendChild(flags);

                if (!editMode) {
                    item.addEventListener('click', () => { selectEvent(event.event_id); });
                }

                listEl.appendChild(item);
            });
        }

        async function selectEvent(eventId) {
            selectedEventId = eventId;
            renderEventList();

            try {
                const resp = await fetch(`/api/events/${eventId}`);
                if (!resp.ok) {
                    throw new Error('Event not found');
                }
                const data = await resp.json();
                renderEventDetail(data);
            } catch (e) {
                console.error('Failed to load event detail', e);
            }
        }

        function renderEventDetail(event) {
            document.getElementById('no-event-selected').style.display = 'none';
            document.getElementById('event-detail').style.display = 'block';
            document.getElementById('delete-event-btn').style.display = 'block';
            document.getElementById('delete-event-btn').setAttribute('data-event-id', event.event_id);

            document.getElementById('detail-type').textContent = event.event_type || '';
            document.getElementById('detail-device').textContent = event.device_name || '';
            document.getElementById('detail-time').textContent = formatDateTimeLocal(event.event_time);
            document.getElementById('detail-email').textContent = event.email_sent ? 'Ja' : 'Nein';
            document.getElementById('detail-description').textContent = event.description || '—';

            const videoList = document.getElementById('video-list');
            videoList.innerHTML = '';

            const videoFiles = event.video_files || [];
            if (!videoFiles.length) {
                const msg = document.createElement('div');
                msg.style.color = '#888';
                msg.style.fontSize = '0.9em';
                msg.textContent = 'Keine Videoclips für dieses Ereignis verfügbar.';
                videoList.appendChild(msg);
                return;
            }

            videoFiles.forEach(vf => {
                const card = document.createElement('div');
                card.className = 'video-card';

                const header = document.createElement('div');
                header.className = 'video-card-header';

                const title = document.createElement('h4');
                title.textContent = `Kamera: ${vf.camera_id}`;
                title.style.margin = '0';
                header.appendChild(title);

                const downloadBtn = document.createElement('button');
                downloadBtn.className = 'download-btn';
                downloadBtn.innerHTML = '⬇️ Download';
                downloadBtn.onclick = () => downloadVideo(event.event_id, vf.camera_id, event.event_time);
                header.appendChild(downloadBtn);

                card.appendChild(header);

                const video = document.createElement('video');
                video.controls = true;
                video.preload = 'metadata';
                video.style.width = '100%';
                video.style.maxHeight = '400px';
                video.src = `/api/events/${event.event_id}/video/${vf.camera_id}`;
                video.onerror = function() {
                    console.error(`Failed to load video for camera ${vf.camera_id}`);
                    const errorMsg = document.createElement('div');
                    errorMsg.style.color = '#ff9800';
                    errorMsg.style.fontSize = '0.9em';
                    errorMsg.textContent = `Video konnte nicht geladen werden für ${vf.camera_id}`;
                    card.appendChild(errorMsg);
                };
                card.appendChild(video);

                videoList.appendChild(card);
            });
        }

        function changeDay(delta) {
            if (!selectedDate) {
                const todayKey = dateKeyFromIso(new Date().toISOString());
                selectedDate = todayKey;
            }
            const base = new Date(selectedDate + 'T00:00:00');
            base.setDate(base.getDate() + delta);
            const pad = (n) => String(n).padStart(2, '0');
            selectedDate = `${base.getFullYear()}-${pad(base.getMonth() + 1)}-${pad(base.getDate())}`;
            updateDateUi();
            renderEventList();
        }

        function onDatePicked(selectedDates, dateStr, instance) {
            if (!dateStr) return;
            selectedDate = dateStr;
            updateDateUi();
            renderEventList();
        }

        function initDatePicker() {
            flatpickrInstance = flatpickr('#date-picker', {
                dateFormat: 'Y-m-d',
                locale: {
                    firstDayOfWeek: 1,
                    weekdays: {
                        shorthand: ['So', 'Mo', 'Di', 'Mi', 'Do', 'Fr', 'Sa'],
                        longhand: ['Sonntag', 'Montag', 'Dienstag', 'Mittwoch', 'Donnerstag', 'Freitag', 'Samstag']
                    },
                    months: {
                        shorthand: ['Jan', 'Feb', 'Mär', 'Apr', 'Mai', 'Jun', 'Jul', 'Aug', 'Sep', 'Okt', 'Nov', 'Dez'],
                        longhand: ['Januar', 'Februar', 'März', 'April', 'Mai', 'Juni', 'Juli', 'August', 'September', 'Oktober', 'November', 'Dezember']
                    }
                },
                onChange: onDatePicked,
                onMonthChange: function() {
                    // Update indicators when month changes
                    setTimeout(updateCalendarEventIndicators, 100);
                },
                onYearChange: function() {
                    // Update indicators when year changes
                    setTimeout(updateCalendarEventIndicators, 100);
                },
                onReady: function(selectedDates, dateStr, instance) {
                    updateCalendarEventIndicators();
                }
            });
        }

        // Handle realtime updates from backend via WebSocket (broadcast from SecurityAlertHandler)
        function initSecurityEventsWebSocket() {
            try {
                const ws = new WebSocket(`ws://${window.location.host}/ws/position`);
                ws.onmessage = (event) => {
                    try {
                        const msg = JSON.parse(event.data);
                        if (msg.type === 'security_event') {
                            const ev = msg.data || {};
                            // Avoid duplicates
                            if (allEvents.some(e => e.event_id === ev.event_id)) {
                                return;
                            }
                            allEvents.unshift(ev);
                            const key = dateKeyFromIso(ev.event_time);
                            if (key) {
                                datesWithEvents.add(key);
                            }
                            // If no date selected yet, or we're on the same day, refresh UI
                            if (!selectedDate || key === selectedDate) {
                                if (!selectedDate) {
                                    selectedDate = key;
                                }
                                updateDateUi();
                                renderEventList();
                            } else {
                                // Only update calendar indicators if other day
                                updateDateUi();
                                updateCalendarEventIndicators();
                            }
                        }
                    } catch (e) {
                        console.error('Failed to handle WS security_event:', e);
                    }
                };
            } catch (e) {
                console.error('Failed to init security events WebSocket:', e);
            }
        }

        async function downloadVideo(eventId, cameraId, eventTime) {
            try {
                const videoUrl = `/api/events/${eventId}/video/${cameraId}`;
                
                // Fetch the video with proper headers
                const response = await fetch(videoUrl);
                if (!response.ok) {
                    throw new Error('Failed to download video');
                }

                // Get the blob
                const blob = await response.blob();
                
                // Generate filename in format: YYYY_MM_DD-HH_MM-camera_id.ext
                let filename = `${cameraId}.mp4`;
                if (eventTime) {
                    try {
                        const d = parseEventDateTime(eventTime);
                        if (d && !isNaN(d.getTime())) {
                            const pad = (n) => String(n).padStart(2, '0');
                            const dateStr = `${d.getFullYear()}_${pad(d.getMonth() + 1)}_${pad(d.getDate())}`;
                            const timeStr = `${pad(d.getHours())}_${pad(d.getMinutes())}`;
                            // Get file extension from Content-Disposition or default to .mp4
                            const contentDisposition = response.headers.get('Content-Disposition');
                            let ext = '.mp4';
                            if (contentDisposition) {
                                const extMatch = contentDisposition.match(/filename="[^"]+\.(\w+)"/);
                                if (extMatch) {
                                    ext = '.' + extMatch[1];
                                }
                            }
                            filename = `${dateStr}-${timeStr}-${cameraId}${ext}`;
                        }
                    } catch (e) {
                        console.warn('Failed to parse event time for filename', e);
                    }
                }
                
                // Create a blob URL
                const blobUrl = window.URL.createObjectURL(blob);
                
                // Create a temporary anchor element to trigger download
                const a = document.createElement('a');
                a.href = blobUrl;
                a.download = filename;
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                
                // Clean up the blob URL
                window.URL.revokeObjectURL(blobUrl);
            } catch (e) {
                console.error('Failed to download video', e);
                alert('Fehler beim Herunterladen des Videos: ' + e.message);
            }
        }

        function toggleEditMode() {
            editMode = !editMode;
            const btn = document.getElementById('edit-mode-btn');
            const controls = document.getElementById('edit-mode-controls');
            
            if (editMode) {
                btn.classList.add('active');
                btn.textContent = '✓ Fertig';
                controls.classList.remove('hidden');
                selectedEventIds.clear();
                // Clear selection when entering edit mode
                selectedEventId = null;
                document.getElementById('no-event-selected').style.display = 'block';
                document.getElementById('event-detail').style.display = 'none';
            } else {
                btn.classList.remove('active');
                btn.textContent = '✏️ Bearbeiten';
                controls.classList.add('hidden');
                selectedEventIds.clear();
            }
            
            updateSelectedCount();
            renderEventList();
        }

        function updateSelectedCount() {
            const countEl = document.getElementById('selected-count');
            const count = selectedEventIds.size;
            countEl.textContent = `${count} ausgewählt`;
        }

        async function deleteSelectedEvents() {
            const ids = Array.from(selectedEventIds);
            if (ids.length === 0) {
                alert('Bitte wählen Sie mindestens ein Ereignis aus.');
                return;
            }

            if (!confirm(`Möchten Sie wirklich ${ids.length} Ereignis(se) löschen?`)) {
                return;
            }

            let successCount = 0;
            let failCount = 0;
            const failedIds = [];

            for (const eventId of ids) {
                try {
                    const resp = await fetch(`/api/events/${eventId}`, {
                        method: 'DELETE',
                    });
                    if (!resp.ok) {
                        throw new Error('Failed to delete');
                    }
                    successCount++;
                } catch (e) {
                    failCount++;
                    failedIds.push(eventId);
                    console.error(`Failed to delete event ${eventId}:`, e);
                }
            }

            // Remove deleted events from local list
            const deletedKeys = new Set();
            ids.forEach(eventId => {
                const deletedEvent = allEvents.find(e => e.event_id === eventId);
                if (deletedEvent) {
                    const key = dateKeyFromIso(deletedEvent.event_time);
                    if (key) {
                        deletedKeys.add(key);
                    }
                }
            });

            allEvents = allEvents.filter(e => !ids.includes(e.event_id));
            totalEvents = Math.max(0, totalEvents - ids.length);
            currentOffset = allEvents.length;

            // Update datesWithEvents
            deletedKeys.forEach(key => {
                const hasOtherEvents = allEvents.some(e => {
                    const k = dateKeyFromIso(e.event_time);
                    return k === key;
                });
                if (!hasOtherEvents) {
                    datesWithEvents.delete(key);
                }
            });

            // Clear selection
            selectedEventIds.clear();
            updateSelectedCount();

            // Refresh UI
            updateDateUi();
            renderEventList();
            updateLoadMoreButton();
            updateCalendarEventIndicators();

            if (failCount > 0) {
                alert(`✅ ${successCount} Ereignis(se) gelöscht.\n❌ ${failCount} Ereignis(se) konnten nicht gelöscht werden.`);
            } else {
                alert(`✅ ${successCount} Ereignis(se) erfolgreich gelöscht.`);
            }
        }

        function showDownloadProgress() {
            document.getElementById('download-progress-modal').classList.add('active');
            downloadCancelled = false;
        }

        function hideDownloadProgress() {
            document.getElementById('download-progress-modal').classList.remove('active');
        }

        function updateDownloadProgress(current, total, currentFile) {
            downloadProgress.current = current;
            downloadProgress.total = total;
            downloadProgress.currentFile = currentFile;
            
            const percentage = total > 0 ? Math.round((current / total) * 100) : 0;
            const progressBar = document.getElementById('download-progress-bar');
            const progressText = document.getElementById('download-progress-text');
            const progressDetails = document.getElementById('download-progress-details');
            
            progressBar.style.width = `${percentage}%`;
            progressBar.textContent = total > 0 ? `${percentage}%` : '';
            progressText.textContent = currentFile || 'Vorbereitung...';
            progressDetails.textContent = `${current} / ${total}`;
        }

        function cancelDownload() {
            downloadCancelled = true;
            hideDownloadProgress();
        }

        async function downloadAllVideosFromSelected() {
            const ids = Array.from(selectedEventIds);
            if (ids.length === 0) {
                alert('Bitte wählen Sie mindestens ein Ereignis aus.');
                return;
            }

            // Collect all events with their videos
            const eventsWithVideos = [];
            for (const eventId of ids) {
                try {
                    const resp = await fetch(`/api/events/${eventId}`);
                    if (resp.ok) {
                        const event = await resp.json();
                        if (event.video_files && event.video_files.length > 0) {
                            eventsWithVideos.push(event);
                        }
                    }
                } catch (e) {
                    console.error(`Failed to load event ${eventId}:`, e);
                }
            }

            if (eventsWithVideos.length === 0) {
                alert('Keine Videos in den ausgewählten Ereignissen gefunden.');
                return;
            }

            // Count total videos
            let totalVideos = 0;
            eventsWithVideos.forEach(event => {
                totalVideos += event.video_files.length;
            });

            // Show progress modal
            showDownloadProgress();
            updateDownloadProgress(0, totalVideos, 'Starte Downloads...');

            // Download all videos
            let downloadCount = 0;
            let failCount = 0;
            let currentIndex = 0;

            for (const event of eventsWithVideos) {
                if (downloadCancelled) break;
                
                for (const vf of event.video_files) {
                    if (downloadCancelled) break;
                    
                    currentIndex++;
                    const fileName = `${formatDateTimeLocal(event.event_time)} - ${vf.camera_id}`;
                    updateDownloadProgress(currentIndex, totalVideos, `Lade: ${fileName}`);
                    
                    try {
                        await downloadVideo(event.event_id, vf.camera_id, event.event_time);
                        downloadCount++;
                        // Small delay to avoid overwhelming the browser
                        await new Promise(resolve => setTimeout(resolve, 200));
                    } catch (e) {
                        failCount++;
                        console.error(`Failed to download video ${vf.camera_id} from event ${event.event_id}:`, e);
                    }
                }
            }

            hideDownloadProgress();

            if (downloadCancelled) {
                alert(`Download abgebrochen. ${downloadCount} Video(s) wurden bereits heruntergeladen.`);
            } else if (failCount > 0) {
                alert(`✅ ${downloadCount} Video(s) heruntergeladen.\n❌ ${failCount} Video(s) konnten nicht heruntergeladen werden.`);
            } else {
                alert(`✅ ${downloadCount} Video(s) erfolgreich heruntergeladen.`);
            }
        }

        async function deleteCurrentEvent() {
            const btn = document.getElementById('delete-event-btn');
            const eventId = btn.getAttribute('data-event-id');
            if (!eventId) return;

            if (!confirm(`Möchten Sie dieses Ereignis wirklich löschen?`)) {
                return;
            }

            try {
                const resp = await fetch(`/api/events/${eventId}`, {
                    method: 'DELETE',
                });
                if (!resp.ok) {
                    const data = await resp.json();
                    throw new Error(data.detail || 'Failed to delete event');
                }

                // Find the event's date before removing it
                const deletedEvent = allEvents.find(e => e.event_id === eventId);
                const key = deletedEvent ? dateKeyFromIso(deletedEvent.event_time) : null;

                // Remove from local list
                allEvents = allEvents.filter(e => e.event_id !== eventId);
                totalEvents = Math.max(0, totalEvents - 1);
                currentOffset = allEvents.length;

                // Update datesWithEvents if no events remain for this date
                if (key) {
                    const hasOtherEvents = allEvents.some(e => {
                        const k = dateKeyFromIso(e.event_time);
                        return k === key;
                    });
                    if (!hasOtherEvents) {
                        datesWithEvents.delete(key);
                    }
                }

                // Clear detail view
                document.getElementById('no-event-selected').style.display = 'block';
                document.getElementById('event-detail').style.display = 'none';
                document.getElementById('delete-event-btn').style.display = 'none';
                selectedEventId = null;

                // Refresh UI
                updateDateUi();
                renderEventList();
                updateCalendarEventIndicators();
            } catch (e) {
                console.error('Failed to delete event', e);
                alert('Fehler beim Löschen des Ereignisses: ' + e.message);
            }
        }

        // Initialize date picker immediately
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', () => {
                initDatePicker();
            });
        } else {
            initDatePicker();
        }
        
        // Initialize page
        try {
            loadEvents();
            initSecurityEventsWebSocket();
        } catch (error) {
            console.error('Error initializing events page:', error);
        }

        // Time Window Management
        let timeWindows = [];
        let alwaysEnableEventTypes = [];
        let emailNotificationTimeout = 60; // seconds, default 60
        let videoCaptureTimeout = 60; // seconds, default 60

        const WEEKDAYS = ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday'];
        const WEEKDAY_LABELS = {
            'monday': 'Montag',
            'tuesday': 'Dienstag',
            'wednesday': 'Mittwoch',
            'thursday': 'Donnerstag',
            'friday': 'Freitag',
            'saturday': 'Samstag',
            'sunday': 'Sonntag'
        };

        function openTimeWindowModal() {
            loadTimeWindows();
            document.getElementById('time-window-modal').classList.add('active');
        }

        function closeTimeWindowModal() {
            document.getElementById('time-window-modal').classList.remove('active');
        }

        async function loadTimeWindows() {
            try {
                const response = await fetch('/api/events/settings/security');
                const data = await response.json();
                
                timeWindows = data.notification_windows || [];
                if (!Array.isArray(timeWindows)) {
                    timeWindows = [];
                }
                
                alwaysEnableEventTypes = data.always_enable_event_types || [];
                if (!Array.isArray(alwaysEnableEventTypes)) {
                    alwaysEnableEventTypes = [];
                }
                
                // Load timeout values
                emailNotificationTimeout = data.email_notification_timeout_seconds || 60;
                videoCaptureTimeout = data.video_capture_timeout_seconds || 60;
                
                // Clamp values to valid range (0-300)
                emailNotificationTimeout = Math.max(0, Math.min(300, emailNotificationTimeout));
                videoCaptureTimeout = Math.max(0, Math.min(300, videoCaptureTimeout));
                
                // Update UI
                document.getElementById('email-notification-timeout').value = emailNotificationTimeout;
                document.getElementById('video-capture-timeout').value = videoCaptureTimeout;
                updateTimeoutDisplay('email-notification-timeout', 'email-notification-timeout-display');
                updateTimeoutDisplay('video-capture-timeout', 'video-capture-timeout-display');
                
                updateAlwaysEnableButtons();
                renderTimeWindowList();
            } catch (error) {
                console.error('Error loading time windows:', error);
                timeWindows = [];
                alwaysEnableEventTypes = [];
                emailNotificationTimeout = 60;
                videoCaptureTimeout = 60;
                renderTimeWindowList();
            }
        }

        function updateAlwaysEnableButtons() {
            document.getElementById('always-enable-fire').classList.toggle('active', alwaysEnableEventTypes.includes('fire'));
            document.getElementById('always-enable-person').classList.toggle('active', alwaysEnableEventTypes.includes('person'));
        }

        function updateTimeoutDisplay(inputId, displayId) {
            const input = document.getElementById(inputId);
            const display = document.getElementById(displayId);
            if (input && display) {
                const value = parseInt(input.value);
                display.textContent = value;
                
                // Update global variables
                if (inputId === 'email-notification-timeout') {
                    emailNotificationTimeout = value;
                } else if (inputId === 'video-capture-timeout') {
                    videoCaptureTimeout = value;
                }
            }
        }

        function toggleAlwaysEnable(eventType) {
            const index = alwaysEnableEventTypes.indexOf(eventType);
            if (index > -1) {
                alwaysEnableEventTypes.splice(index, 1);
            } else {
                alwaysEnableEventTypes.push(eventType);
            }
            updateAlwaysEnableButtons();
        }

        function addTimeWindow() {
            timeWindows.push({
                start_day: 'monday',
                start_time: '22:00',
                end_day: 'tuesday',
                end_time: '06:00',
                enabled_event_types: [],
                active: true
            });
            renderTimeWindowList();
            // Expand the newly added window (last one)
            const items = document.querySelectorAll('.time-window-item');
            if (items.length > 0) {
                items[items.length - 1].classList.add('expanded');
            }
        }

        function removeTimeWindow(index) {
            timeWindows.splice(index, 1);
            renderTimeWindowList();
        }

        function toggleWindowActive(index) {
            timeWindows[index].active = !timeWindows[index].active;
            renderTimeWindowList();
        }

        function updateWindowField(index, field, value) {
            timeWindows[index][field] = value;
        }

        function updateWindowSummary(index) {
            const items = document.querySelectorAll('.time-window-item');
            if (index >= items.length) return;
            
            const item = items[index];
            const window = timeWindows[index];
            const summaryText = item.querySelector('.time-window-summary-text');
            if (!summaryText) return;
            
            const startDayLabel = WEEKDAY_LABELS[window.start_day] || window.start_day;
            const endDayLabel = WEEKDAY_LABELS[window.end_day] || window.end_day;
            const eventTypes = (window.enabled_event_types || []);
            const eventTypesStr = eventTypes.length > 0 
                ? eventTypes.map(t => t === 'person' ? '👤 Person' : '🔥 Feuer').join(', ')
                : 'Keine Ereignistypen';
            
            summaryText.innerHTML = `<strong>${startDayLabel} ${window.start_time || ''}</strong> - <strong>${endDayLabel} ${window.end_time || ''}</strong> | ${eventTypesStr}`;
        }

        function toggleWindowEventType(index, eventType) {
            const types = timeWindows[index].enabled_event_types || [];
            const typeIndex = types.indexOf(eventType);
            if (typeIndex > -1) {
                types.splice(typeIndex, 1);
            } else {
                types.push(eventType);
            }
            timeWindows[index].enabled_event_types = types;
            updateWindowSummary(index);
        }

        function renderTimeWindowList() {
            const listEl = document.getElementById('time-window-list');
            listEl.innerHTML = '';

            if (timeWindows.length === 0) {
                const empty = document.createElement('div');
                empty.style.color = '#888';
                empty.style.fontSize = '0.9em';
                empty.textContent = 'Keine Zeitfenster konfiguriert. Klicken Sie auf "+ Zeitfenster hinzufügen" um eines zu erstellen.';
                listEl.appendChild(empty);
                return;
            }

            timeWindows.forEach((window, index) => {
                const item = document.createElement('div');
                item.className = 'time-window-item' + (window.active ? '' : ' inactive');
                // Items are collapsed by default
                
                // Create header (always visible, clickable)
                const header = document.createElement('div');
                header.className = 'time-window-header';
                header.onclick = (e) => {
                    // Don't toggle if clicking on checkbox or delete button
                    if (!e.target.closest('.time-window-checkbox') && !e.target.closest('.time-window-actions')) {
                        item.classList.toggle('expanded');
                    }
                };
                
                const headerContent = document.createElement('div');
                headerContent.className = 'time-window-header-content';
                
                // Active checkbox in header
                const checkbox = document.createElement('div');
                checkbox.className = 'time-window-checkbox';
                const activeCheckbox = document.createElement('input');
                activeCheckbox.type = 'checkbox';
                activeCheckbox.checked = window.active;
                activeCheckbox.onchange = (e) => {
                    e.stopPropagation();
                    toggleWindowActive(index);
                };
                const activeLabel = document.createElement('label');
                activeLabel.textContent = 'Aktiv';
                activeLabel.style.cursor = 'pointer';
                activeLabel.onclick = (e) => {
                    e.stopPropagation();
                    activeCheckbox.click();
                };
                checkbox.appendChild(activeCheckbox);
                checkbox.appendChild(activeLabel);
                
                // Summary text
                const summary = document.createElement('div');
                summary.className = 'time-window-summary';
                const summaryText = document.createElement('div');
                summaryText.className = 'time-window-summary-text';
                
                const startDayLabel = WEEKDAY_LABELS[window.start_day] || window.start_day;
                const endDayLabel = WEEKDAY_LABELS[window.end_day] || window.end_day;
                const eventTypes = (window.enabled_event_types || []);
                const eventTypesStr = eventTypes.length > 0 
                    ? eventTypes.map(t => t === 'person' ? '👤 Person' : '🔥 Feuer').join(', ')
                    : 'Keine Ereignistypen';
                
                summaryText.innerHTML = `<strong>${startDayLabel} ${window.start_time || ''}</strong> - <strong>${endDayLabel} ${window.end_time || ''}</strong> | ${eventTypesStr}`;
                summary.appendChild(summaryText);
                
                headerContent.appendChild(checkbox);
                headerContent.appendChild(summary);
                
                // Toggle arrow
                const toggle = document.createElement('span');
                toggle.className = 'time-window-toggle';
                toggle.textContent = '▶';
                
                header.appendChild(headerContent);
                header.appendChild(toggle);
                item.appendChild(header);
                
                // Create body (collapsible content)
                const body = document.createElement('div');
                body.className = 'time-window-body';
                
                const content = document.createElement('div');
                content.className = 'time-window-content';
                
                // Start Day
                const startDayField = document.createElement('div');
                startDayField.className = 'time-window-field';
                const startDayLabelEl = document.createElement('label');
                startDayLabelEl.textContent = 'Start Tag';
                const startDaySelect = document.createElement('select');
                WEEKDAYS.forEach(day => {
                    const option = document.createElement('option');
                    option.value = day;
                    option.textContent = WEEKDAY_LABELS[day];
                    option.selected = window.start_day === day;
                    startDaySelect.appendChild(option);
                });
                startDaySelect.onchange = (e) => {
                    updateWindowField(index, 'start_day', e.target.value);
                    updateWindowSummary(index);
                };
                startDayField.appendChild(startDayLabelEl);
                startDayField.appendChild(startDaySelect);
                
                // Start Time
                const startTimeField = document.createElement('div');
                startTimeField.className = 'time-window-field';
                const startTimeLabel = document.createElement('label');
                startTimeLabel.textContent = 'Start Zeit';
                const startTimeInput = document.createElement('input');
                startTimeInput.type = 'time';
                startTimeInput.value = window.start_time || '22:00';
                startTimeInput.onchange = (e) => {
                    updateWindowField(index, 'start_time', e.target.value);
                    updateWindowSummary(index);
                };
                startTimeField.appendChild(startTimeLabel);
                startTimeField.appendChild(startTimeInput);
                
                // End Day
                const endDayField = document.createElement('div');
                endDayField.className = 'time-window-field';
                const endDayLabelEl = document.createElement('label');
                endDayLabelEl.textContent = 'Ende Tag';
                const endDaySelect = document.createElement('select');
                WEEKDAYS.forEach(day => {
                    const option = document.createElement('option');
                    option.value = day;
                    option.textContent = WEEKDAY_LABELS[day];
                    option.selected = window.end_day === day;
                    endDaySelect.appendChild(option);
                });
                endDaySelect.onchange = (e) => {
                    updateWindowField(index, 'end_day', e.target.value);
                    updateWindowSummary(index);
                };
                endDayField.appendChild(endDayLabelEl);
                endDayField.appendChild(endDaySelect);
                
                // End Time
                const endTimeField = document.createElement('div');
                endTimeField.className = 'time-window-field';
                const endTimeLabel = document.createElement('label');
                endTimeLabel.textContent = 'Ende Zeit';
                const endTimeInput = document.createElement('input');
                endTimeInput.type = 'time';
                endTimeInput.value = window.end_time || '06:00';
                endTimeInput.onchange = (e) => {
                    updateWindowField(index, 'end_time', e.target.value);
                    updateWindowSummary(index);
                };
                endTimeField.appendChild(endTimeLabel);
                endTimeField.appendChild(endTimeInput);
                
                content.appendChild(startDayField);
                content.appendChild(startTimeField);
                content.appendChild(endDayField);
                content.appendChild(endTimeField);
                
                // Event Types
                const eventTypesDiv = document.createElement('div');
                eventTypesDiv.className = 'time-window-event-types';
                eventTypesDiv.style.gridColumn = '1 / -1';
                
                const personCheckbox = document.createElement('input');
                personCheckbox.type = 'checkbox';
                personCheckbox.id = `window-${index}-person`;
                personCheckbox.checked = (window.enabled_event_types || []).includes('person');
                personCheckbox.onchange = () => toggleWindowEventType(index, 'person');
                const personLabel = document.createElement('label');
                personLabel.htmlFor = `window-${index}-person`;
                personLabel.style.cursor = 'pointer';
                personLabel.appendChild(personCheckbox);
                personLabel.appendChild(document.createTextNode(' 👤 Person'));
                
                const fireCheckbox = document.createElement('input');
                fireCheckbox.type = 'checkbox';
                fireCheckbox.id = `window-${index}-fire`;
                fireCheckbox.checked = (window.enabled_event_types || []).includes('fire');
                fireCheckbox.onchange = () => toggleWindowEventType(index, 'fire');
                const fireLabel = document.createElement('label');
                fireLabel.htmlFor = `window-${index}-fire`;
                fireLabel.style.cursor = 'pointer';
                fireLabel.appendChild(fireCheckbox);
                fireLabel.appendChild(document.createTextNode(' 🔥 Feuer'));
                
                eventTypesDiv.appendChild(personLabel);
                eventTypesDiv.appendChild(fireLabel);
                content.appendChild(eventTypesDiv);
                
                body.appendChild(content);
                
                // Actions in body
                const actions = document.createElement('div');
                actions.className = 'time-window-actions';
                actions.style.marginTop = '15px';
                actions.style.display = 'flex';
                actions.style.justifyContent = 'flex-end';
                const deleteBtn = document.createElement('button');
                deleteBtn.className = 'btn-delete-window';
                deleteBtn.textContent = '🗑️ Löschen';
                deleteBtn.onclick = () => removeTimeWindow(index);
                deleteBtn.title = 'Löschen';
                actions.appendChild(deleteBtn);
                body.appendChild(actions);
                
                item.appendChild(body);
                listEl.appendChild(item);
            });
        }

        async function saveTimeWindows() {
            try {
                // Validate that each time window has at least one event type
                const windowsWithoutEventTypes = [];
                timeWindows.forEach((window, index) => {
                    const eventTypes = window.enabled_event_types || [];
                    if (eventTypes.length === 0) {
                        const startDayLabel = WEEKDAY_LABELS[window.start_day] || window.start_day;
                        const endDayLabel = WEEKDAY_LABELS[window.end_day] || window.end_day;
                        windowsWithoutEventTypes.push(`${startDayLabel} ${window.start_time || ''} - ${endDayLabel} ${window.end_time || ''}`);
                    }
                });

                if (windowsWithoutEventTypes.length > 0) {
                    const windowList = windowsWithoutEventTypes.map((w, i) => `${i + 1}. ${w}`).join('\n');
                    alert(`❌ Bitte wählen Sie mindestens einen Ereignistyp (Person oder Feuer) für jedes Zeitfenster aus:\n\n${windowList}`);
                    return;
                }

                // Load current settings to preserve email settings
                const currentResponse = await fetch('/api/events/settings/security');
                const currentData = await currentResponse.json();
                
                // Get timeout values from UI
                const emailTimeoutInput = document.getElementById('email-notification-timeout');
                const videoTimeoutInput = document.getElementById('video-capture-timeout');
                const emailTimeout = emailTimeoutInput ? parseInt(emailTimeoutInput.value) : emailNotificationTimeout;
                const videoTimeout = videoTimeoutInput ? parseInt(videoTimeoutInput.value) : videoCaptureTimeout;
                
                // Validate timeout values (0-300 seconds)
                const validatedEmailTimeout = Math.max(0, Math.min(300, emailTimeout));
                const validatedVideoTimeout = Math.max(0, Math.min(300, videoTimeout));
                
                const payload = {
                    email: currentData.email || {},
                    notification_windows: timeWindows,
                    always_enable_event_types: alwaysEnableEventTypes,
                    email_notification_timeout_seconds: validatedEmailTimeout,
                    video_capture_timeout_seconds: validatedVideoTimeout,
                    event_defaults: currentData.event_defaults || {
                        pre_event_seconds: 10,
                        post_event_seconds: 10
                    }
                };

                const response = await fetch('/api/events/settings/security', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload)
                });

                if (!response.ok) {
                    const errorData = await response.json().catch(() => ({ detail: 'Unknown error' }));
                    throw new Error(errorData.detail || `HTTP ${response.status}`);
                }

                const data = await response.json();
                if (data.status === 'success') {
                    alert('✅ Zeitfenster erfolgreich gespeichert!');
                    closeTimeWindowModal();
                } else {
                    throw new Error(data.message || 'Unknown error');
                }
            } catch (error) {
                console.error('Error saving time windows:', error);
                alert('❌ Fehler beim Speichern der Zeitfenster: ' + error.message);
            }
        }

        // Close modal on overlay click - wait for DOM to be ready
        function setupModalClickListener() {
            try {
                const modal = document.getElementById('time-window-modal');
                if (modal) {
                    modal.addEventListener('click', function(e) {
                        if (e.target === this) {
                            closeTimeWindowModal();
                        }
                    });
                }
            } catch (error) {
                console.error('Error setting up modal click listener:', error);
            }
        }

        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', setupModalClickListener);
        } else {
            setupModalClickListener();
        }
