# oled_display.py
# Combined QR generation and I2C OLED display for the drone

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image
import qrcode
import json
import io
import threading

class OLEDDisplay:
    def __init__(self, i2c_port=1, i2c_address=0x3C):
        """
        Initialize the OLED display.
        """
        self.serial = i2c(port=i2c_port, address=i2c_address)
        self.device = ssd1306(self.serial)
        self.clear()
    
    def generate_qr_image(self, url: str, api_key: str = None, size: int = 128) -> Image.Image:
        """
        Generate a QR code PIL Image for the given URL and optional API key.
        :param url: Tunnel or public URL for the drone
        :param api_key: Optional API key for secure commands
        :param size: Pixel size for the QR code
        :return: PIL Image object of the QR code
        """
        payload = {"url": url}
        if api_key:
            payload["api_key"] = api_key

        qr = qrcode.QRCode(
            version=1,
            box_size=4,
            border=2
        )
        qr.add_data(json.dumps(payload))
        qr.make(fit=True)
        
        img = qr.make_image(fill_color="black", back_color="white")
        # Resize to fit OLED screen
        img = img.resize((size, size))
        return img

    def show_qr(self, url: str, api_key: str = None):
        """
        Generate and display a QR code for the given URL and API key.
        :param url: Tunnel URL
        :param api_key: Optional API key
        """
        try:
            img = self.generate_qr_image(url, api_key)
            img = img.convert("1")  # Convert to 1-bit monochrome for OLED
            self.device.display(img)
        except Exception as e:
            print(f"[OLED] Error displaying QR: {e}")

    def clear(self):
        """
        Clear the OLED screen.
        """
        self.device.clear()
        
    def show_qr_desktop(self, url: str, api_key: str = None):
        """
        Display QR code in a desktop window for testing without OLED.
        Does not affect the OLED display methods.
        """


        def _show():
            img = self.generate_qr_image(url, api_key)
            img.show()  # opens in default image viewer

        threading.Thread(target=_show, daemon=True).start()
        
    def show_battery(self, percentage: float):
        """
        Display battery percentage and status bar on OLED.
        Called when drone connects to replace QR code.
        """
        from PIL import ImageDraw, ImageFont
    
        # Create blank image
        img = Image.new("1", (self.device.width, self.device.height), 0)
        draw = ImageDraw.Draw(img)
    
        # Title
        draw.text((0, 0), "BATTERY", fill=1)
    
        # Percentage text
        draw.text((0, 20), f"{percentage:.0f}%", fill=1)
    
        # Status bar outline
        bar_x, bar_y = 0, 45
        bar_w, bar_h = 120, 12
        draw.rectangle([bar_x, bar_y, bar_x + bar_w, bar_y + bar_h], outline=1)
    
        # Status bar fill
        fill_w = int((percentage / 100.0) * (bar_w - 2))
        if fill_w > 0:
            draw.rectangle([bar_x + 1, bar_y + 1, bar_x + 1 + fill_w, bar_y + bar_h - 1], fill=1)
    
        # Battery level label
        if percentage > 50:
            label = "GOOD"
        elif percentage > 20:
            label = "LOW"
        else:
            label = "CRITICAL"
        draw.text((0, 58), label, fill=1)
    
        try:
            self.device.display(img)
        except Exception as e:
            print(f"[OLED] Error displaying battery: {e}")
