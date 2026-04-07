"""Email sending for security alerts."""

import os
import smtplib
from dataclasses import dataclass
from email.message import EmailMessage
from typing import List, Tuple


@dataclass
class SecurityEmailSettings:
    enabled: bool = False
    smtp_host: str = ""
    smtp_port: int = 587
    use_tls: bool = True
    username: str = ""
    password: str = ""
    sender: str = ""
    recipients: List[str] = None


class EmailNotifier:
    """Responsible only for sending emails."""

    def __init__(self, settings: SecurityEmailSettings):
        self._settings = settings

    def send_security_alert(self, subject: str, body: str) -> Tuple[bool, str]:
        settings = self._settings
        if not settings.enabled:
            return False, "Email sending disabled"
        if not settings.smtp_host or not settings.sender or not settings.recipients:
            return False, "Email settings incomplete"

        # Allow overriding credentials via environment for security
        username = os.getenv("SECURITY_EMAIL_USER", settings.username)
        password = os.getenv("SECURITY_EMAIL_PASS", settings.password)

        msg = EmailMessage()
        msg["Subject"] = subject
        msg["From"] = settings.sender
        msg["To"] = ", ".join(settings.recipients)
        msg.set_content(body)

        try:
            with smtplib.SMTP(settings.smtp_host, settings.smtp_port, timeout=10) as smtp:
                if settings.use_tls:
                    smtp.starttls()
                if username:
                    smtp.login(username, password)
                smtp.send_message(msg)
            return True, "Email sent"
        except Exception as e:
            return False, str(e)


