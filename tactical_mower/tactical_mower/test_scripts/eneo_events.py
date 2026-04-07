#!/usr/bin/env python3
"""
Eneo-Event UDP Parser
=====================
Lauscht auf UDP Port 5002 für Eneo-Kamera Events (Broadcast)

JSON Struktur:
{
    "result": "success",
    "data": {
        "ChannelName": "Eneo-Event",
        "DeviceName": "INT-8SF0003M0A",
        "IPAddress": "192.168.1.10",
        "MacAddress": "60-27-1C-09-58-9D",
        "alarm_list": [
            {
                "EventType": "FireDetect",      # Event-Typ (siehe unten)
                "EventTime": "2015-1-5_14:17:53",
                "EventAction": "start",          # "start" oder "stop"
                "Chn": ["CH2"]                   # Betroffene Kanäle
            }
        ]
    }
}

Bekannte EventTypes:
- "FireDetect"      : Feuer erkannt
- "PersonDetect"    : Person erkannt  
- "MotionDetect"    : Bewegung erkannt
- "FaceDetect"      : Gesicht erkannt
- "VehicleDetect"   : Fahrzeug erkannt
- "LineCross"       : Linie überquert
- "RegionEnter"     : Region betreten
- "RegionExit"      : Region verlassen
"""

import socket
import json
from datetime import datetime
from typing import Callable, Optional

# Konfiguration
ENEO_EVENT_PORT = 5002
BUFFER_SIZE = 4096


class EneoEvent:
    """Datenklasse für ein Eneo-Event"""
    def __init__(self, event_type: str, event_time: str, event_action: str, 
                 channels: list, device_name: str, ip_address: str, mac_address: str):
        self.event_type = event_type
        self.event_time = event_time
        self.event_action = event_action
        self.channels = channels
        self.device_name = device_name
        self.ip_address = ip_address
        self.mac_address = mac_address
    
    def __repr__(self):
        return f"EneoEvent({self.event_type}, {self.event_action}, {self.channels})"
    
    @property
    def is_fire(self) -> bool:
        return self.event_type == "FireDetect"
    
    @property
    def is_person(self) -> bool:
        return self.event_type == "PersonDetect"
    
    @property
    def is_motion(self) -> bool:
        return self.event_type == "MotionDetect"
    
    @property
    def is_start(self) -> bool:
        return self.event_action == "start"
    
    @property
    def is_stop(self) -> bool:
        return self.event_action == "stop"


def parse_eneo_json(data: bytes) -> list[EneoEvent]:
    """
    Parst die Eneo-Event JSON-Daten und gibt eine Liste von Events zurück.
    
    Args:
        data: Raw UDP payload bytes
        
    Returns:
        Liste von EneoEvent Objekten
    """
    events = []
    
    try:
        json_data = json.loads(data.decode('utf-8'))
        
        if json_data.get('result') != 'success':
            return events
        
        event_data = json_data.get('data', {})
        device_name = event_data.get('DeviceName', '')
        ip_address = event_data.get('IPAddress', '')
        mac_address = event_data.get('MacAddress', '')
        
        for alarm in event_data.get('alarm_list', []):
            event = EneoEvent(
                event_type=alarm.get('EventType', ''),
                event_time=alarm.get('EventTime', ''),
                event_action=alarm.get('EventAction', ''),
                channels=alarm.get('Chn', []),
                device_name=device_name,
                ip_address=ip_address,
                mac_address=mac_address
            )
            events.append(event)
            
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        print(f"Parse error: {e}")
    
    return events


def listen_for_events(callback: Optional[Callable[[EneoEvent], None]] = None,
                      filter_types: Optional[list[str]] = None):
    """
    Lauscht auf Eneo-Events und ruft callback für jedes Event auf.
    
    Args:
        callback: Funktion die für jedes Event aufgerufen wird
        filter_types: Nur diese EventTypes verarbeiten (z.B. ["FireDetect", "PersonDetect"])
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(('', ENEO_EVENT_PORT))
    
    print(f"Listening for Eneo-Events on UDP port {ENEO_EVENT_PORT}...")
    
    try:
        while True:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            events = parse_eneo_json(data)
            
            for event in events:
                # Filter anwenden
                if filter_types and event.event_type not in filter_types:
                    continue
                
                # Callback aufrufen oder ausgeben
                if callback:
                    callback(event)
                else:
                    print(f"[{datetime.now()}] {event}")
                    
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


# Beispiel-Callbacks
def on_fire_alarm(event: EneoEvent):
    """Wird bei Feuer-Alarm aufgerufen"""
    if event.is_fire and event.is_start:
        print(f"🔥 FEUER ALARM! Kamera: {event.device_name}, Kanal: {event.channels}")

def on_person_detected(event: EneoEvent):
    """Wird bei Personenerkennung aufgerufen"""
    if event.is_person and event.is_start:
        print(f"👤 Person erkannt! Kamera: {event.device_name}, Kanal: {event.channels}")


if __name__ == "__main__":
    # Beispiel: Nur auf Feuer und Personen reagieren
    def handle_event(event: EneoEvent):
        if event.is_fire:
            on_fire_alarm(event)
        elif event.is_person:
            on_person_detected(event)
        else:
            print(f"Event: {event.event_type} - {event.event_action}")
    
    listen_for_events(callback=handle_event)
