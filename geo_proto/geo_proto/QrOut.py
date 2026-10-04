import qrcode
import json
import threading
from PIL import Image, ImageDraw
from luma.core.interface.serial import i2c
from luma.oled.device import sh1107 

class OLEDDisplay:
    def __init__(self, i2c_port=1, i2c_address=0x3C):
        self.width = 128
        self.height = 128
        serial = i2c(port=i2c_port, address=i2c_address)
        self.device = sh1107(serial, width=128, height=128, rotate=3) 
        self.device.cleanup = lambda: None

    def _display_image(self, img): 
        img = img.convert('1') 
        canvas = Image.new('1', (self.width, self.height), 0) 
        x_offset = max(0, (self.width - img.width) // 2) 
        y_offset = max(0, (self.height - img.height) // 2) 
        img = img.crop((0, 0, min(img.width, self.width), min(img.height, self.height))) 
        canvas.paste(img, (x_offset, y_offset)) 
        self.device.display(canvas) 
 
    def generate_qr_image(self, url: str, api_key: str = None, size: int = 120) -> Image.Image: 
        payload = {"url": url} 
        if api_key: 
            payload["api_key"] = api_key 
            qr = qrcode.QRCode( 
            version=None, 
            error_correction=qrcode.constants.ERROR_CORRECT_L, 
            box_size=2, 
            border=1 
        ) 
        qr.add_data(json.dumps(payload)) 
        qr.make(fit=True) 
        img = qr.make_image(fill_color="white", back_color="black") 
        img = img.convert('1').resize((size, size)) 
        return img 
 
    def show_qr(self, url: str, api_key: str = None): 
        try: 
            img = self.generate_qr_image(url, api_key) 
            self._display_image(img) 
        except Exception as e: 
            print(f"[OLED] Error displaying QR: {e}") 
 
    def show_battery(self, percentage: float): 
        img = Image.new('1', (self.width, self.height), 0) 
        draw = ImageDraw.Draw(img) 
        draw.text((0, 0), "BATTERY", fill=1) 
        draw.text((0, 20), f"{percentage:.0f}%", fill=1) 
        bar_w = int((percentage / 100.0) * 120) 
        draw.rectangle([0, 50, 120, 65], outline=1) 
        if bar_w > 0: 
            draw.rectangle([1, 51, bar_w, 64], fill=1) 
        if percentage > 50: 
            label = "GOOD" 
        elif percentage > 20: 
            label = "LOW" 
        else: 
            label = "CRITICAL" 
        draw.text((0, 80), label, fill=1) 
        self._display_image(img) 
 
    def show_qr_desktop(self, url: str, api_key: str = None): 
        def _show(): 
            img = self.generate_qr_image(url, api_key, size=256) 
            img.show() 
        threading.Thread(target=_show, daemon=True).start() 
        
    def show_status(self, message: str):
        #Display a status message on the OLED
        img = Image.new('1', (self.width, self.height), 0)
        draw = ImageDraw.Draw(img)
        
        #Wrap text at ~20 characters per line
        words = message.split()
        lines = []
        current = ""
        for word in words:
            if len(current + "") <= 15:
                current += (" " + word if current else word)
            else:
                lines.append(current)
                current = word
        
        if current:
            lines.append(current)
            
        for i, line in enumerate(lines[:5]): #Max 5 lines
            draw.text((0, i * 20), line, fill=1)
            
        self._display_image(img)
    
    def show_logo(self, subtitle: str = "Starting..."):
        #Display Aeron logo on OLED with sub-text
        img = Image.new('1', (self.width, self.height), 0)
        draw = ImageDraw.Draw(img)
        
        #Main A Shape
        draw.polygon([(64, 5), (20, 75), (108, 75)], outline=1)
        #Inner cutout
        draw.polygon([(64,20), (35, 70), (93, 70)], fill=0)
        #crossbar
        draw.rectangle([42, 52, 86, 58], fill=1)
        #Bottom arrow point
        draw.polygon([(57, 68), (64, 82), (71, 68)], fill=1)
        #Left arm and propeller
        draw.line([(108, 58), (123, 58)], fill=1, width=2)
        draw.ellipse([(2, 53), (126, 63)], outline=1)
        #Aeron Text
        draw.text((18,95), 'A E R O N', fill=1)
        #Subtitle
        draw.text((30, 112), subtitle, fill=1)
        
        self._display_image(img)
        
 
    def clear(self): 
        self.device.clear() 
 
