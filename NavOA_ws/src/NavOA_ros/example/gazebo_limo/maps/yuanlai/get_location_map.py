#!/usr/bin/env python
import yaml
import cv2
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import math


class MapCoordinateGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Map Coordinate Tool")
        self.root.geometry("1200x800")

        self.map_image = None
        self.map_data = None
        self.points = []

        self.create_widgets()

        self.yaml_path = None

    def create_widgets(self):
        control_frame = tk.Frame(self.root, width=300, bg="#f0f0f0")
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        control_frame.pack_propagate(False)

        title_label = tk.Label(control_frame, text="Map Coordinate Tool",
                               font=("Arial", 16, "bold"), bg="#f0f0f0")
        title_label.pack(pady=20)

        load_yaml_btn = tk.Button(control_frame, text="Load Map YAML",
                                  command=self.load_yaml_file,
                                  width=20, height=2, bg="#4CAF50", fg="white")
        load_yaml_btn.pack(pady=10)

        load_img_btn = tk.Button(control_frame, text="Load Map Image",
                                 command=self.load_image_file,
                                 width=20, height=2, bg="#2196F3", fg="white")
        load_img_btn.pack(pady=10)

        self.info_text = tk.Text(control_frame, height=15, width=35)
        self.info_text.pack(pady=10)

        coords_label = tk.Label(control_frame, text="Captured Points:",
                                font=("Arial", 12, "bold"), bg="#f0f0f0")
        coords_label.pack(pady=(20, 5))

        self.coords_listbox = tk.Listbox(control_frame, height=10, width=35)
        self.coords_listbox.pack(pady=5)

        btn_frame = tk.Frame(control_frame, bg="#f0f0f0")
        btn_frame.pack(pady=10)

        clear_btn = tk.Button(btn_frame, text="Clear All",
                              command=self.clear_points,
                              width=12, bg="#ff9800", fg="white")
        clear_btn.pack(side=tk.LEFT, padx=5)

        export_btn = tk.Button(btn_frame, text="Export",
                               command=self.export_coordinates,
                               width=12, bg="#9C27B0", fg="white")
        export_btn.pack(side=tk.LEFT, padx=5)

        image_frame = tk.Frame(self.root, bg="white")
        image_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.canvas = tk.Canvas(image_frame, bg="white", cursor="cross")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Motion>", self.on_mouse_move)

        self.status_label = tk.Label(image_frame, text="Please load map file",
                                     bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.pack(fill=tk.X)

        self.coord_label = tk.Label(image_frame, text="Coord: (0.00, 0.00)",
                                    bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.coord_label.pack(fill=tk.X)

    def load_yaml_file(self):
        file_path = filedialog.askopenfilename(
            title="Select Map YAML",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )

        if file_path:
            try:
                with open(file_path, 'r') as f:
                    self.map_data = yaml.safe_load(f)

                self.yaml_path = file_path

                map_dir = os.path.dirname(file_path)
                image_file = os.path.join(map_dir, self.map_data['image'])

                if os.path.exists(image_file):
                    self.load_map_image(image_file)
                else:
                    messagebox.showwarning("Warning", f"Image not found: {image_file}")

                self.update_info_text()

            except Exception as e:
                messagebox.showerror("Error", f"Load YAML failed: {str(e)}")

    def load_image_file(self):
        if not self.map_data:
            messagebox.showinfo("Info", "Please load YAML first")
            return

        file_path = filedialog.askopenfilename(
            title="Select Map Image",
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.pgm *.bmp"), ("All files", "*.*")]
        )

        if file_path:
            self.load_map_image(file_path)

    def load_map_image(self, image_path):
        try:
            pil_image = Image.open(image_path)

            if pil_image.mode in ('RGBA', 'LA') or (pil_image.mode == 'P' and 'transparency' in pil_image.info):
                rgb_image = Image.new('RGB', pil_image.size, (255, 255, 255))
                rgb_image.paste(pil_image, mask=pil_image.split()[-1])
                pil_image = rgb_image

            self.original_image = pil_image.copy()
            self.display_image = self.resize_image(pil_image)

            self.tk_image = ImageTk.PhotoImage(self.display_image)

            self.canvas.delete("all")
            self.canvas.config(width=self.display_image.width, height=self.display_image.height)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image, tags="map")

            self.status_label.config(text=f"Loaded: {os.path.basename(image_path)}")

        except Exception as e:
            messagebox.showerror("Error", f"Load image failed: {str(e)}")

    def resize_image(self, image, max_width=900, max_height=700):
        width, height = image.size

        width_ratio = max_width / width
        height_ratio = max_height / height
        ratio = min(width_ratio, height_ratio)

        if ratio < 1:
            new_width = int(width * ratio)
            new_height = int(height * ratio)
            return image.resize((new_width, new_height), Image.Resampling.LANCZOS)

        return image

    def update_info_text(self):
        if self.map_data:
            info = f"Map Info:\n"
            info += f"Resolution: {self.map_data['resolution']} m/pixel\n"
            info += f"Origin: {self.map_data['origin']}\n"

            if hasattr(self, 'original_image'):
                width, height = self.original_image.size
                info += f"Image Size: {width} x {height} pixels\n"
                info += f"Map Real Size:\n"
                info += f"  Width: {width * self.map_data['resolution']:.2f} m\n"
                info += f"  Height: {height * self.map_data['resolution']:.2f} m\n"

            self.info_text.delete(1.0, tk.END)
            self.info_text.insert(1.0, info)

    def on_canvas_click(self, event):
        if not self.map_data or not hasattr(self, 'original_image'):
            return

        pixel_x = event.x
        pixel_y = event.y

        display_width = self.display_image.width
        display_height = self.display_image.height
        original_width, original_height = self.original_image.size

        scale_x = original_width / display_width
        scale_y = original_height / display_height

        original_pixel_x = int(pixel_x * scale_x)
        original_pixel_y = int(pixel_y * scale_y)

        resolution = self.map_data['resolution']
        origin_x = self.map_data['origin'][0]
        origin_y = self.map_data['origin'][1]

        world_x = origin_x + original_pixel_x * resolution
        world_y = origin_y + (original_height - original_pixel_y) * resolution

        point_info = {
            'pixel': (original_pixel_x, original_pixel_y),
            'world': (world_x, world_y),
            'canvas': (pixel_x, pixel_y)
        }
        self.points.append(point_info)

        radius = 5
        self.canvas.create_oval(
            pixel_x - radius, pixel_y - radius,
            pixel_x + radius, pixel_y + radius,
            fill='red', outline='white', width=2, tags=f"point_{len(self.points)}"
        )

        self.canvas.create_text(
            pixel_x, pixel_y - 15,
            text=str(len(self.points)),
            fill='yellow', font=('Arial', 12, 'bold'),
            tags=f"label_{len(self.points)}"
        )

        coord_text = f"Point{len(self.points)}: ({world_x:.3f}, {world_y:.3f})"
        self.coords_listbox.insert(tk.END, coord_text)

        self.status_label.config(
            text=f"Point{len(self.points)}: Pixel({original_pixel_x}, {original_pixel_y}) World({world_x:.3f}, {world_y:.3f})")

    def on_mouse_move(self, event):
        if not self.map_data or not hasattr(self, 'original_image'):
            return

        pixel_x = event.x
        pixel_y = event.y

        display_width = self.display_image.width
        display_height = self.display_image.height
        original_width, original_height = self.original_image.size

        scale_x = original_width / display_width
        scale_y = original_height / display_height

        original_pixel_x = int(pixel_x * scale_x)
        original_pixel_y = int(pixel_y * scale_y)

        resolution = self.map_data['resolution']
        origin_x = self.map_data['origin'][0]
        origin_y = self.map_data['origin'][1]

        world_x = origin_x + original_pixel_x * resolution
        world_y = origin_y + (original_height - original_pixel_y) * resolution

        self.coord_label.config(
            text=f"Coord: ({world_x:.3f}, {world_y:.3f}) | Pixel: ({original_pixel_x}, {original_pixel_y})")

    def clear_points(self):
        self.points = []
        self.coords_listbox.delete(0, tk.END)

        for item in self.canvas.find_all():
            tags = self.canvas.gettags(item)
            if any(tag.startswith('point_') or tag.startswith('label_') for tag in tags):
                self.canvas.delete(item)

        self.status_label.config(text="All points cleared")

    def export_coordinates(self):
        if not self.points:
            messagebox.showinfo("Info", "No points to export")
            return

        file_path = filedialog.asksaveasfilename(
            title="Save Coordinates",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("Text files", "*.txt"), ("All files", "*.*")]
        )

        if file_path:
            try:
                export_data = {
                    'map_info': self.map_data,
                    'points': []
                }

                for i, point in enumerate(self.points, 1):
                    export_data['points'].append({
                        'id': i,
                        'world_x': float(f"{point['world'][0]:.3f}"),
                        'world_y': float(f"{point['world'][1]:.3f}"),
                        'pixel_x': point['pixel'][0],
                        'pixel_y': point['pixel'][1]
                    })

                with open(file_path, 'w') as f:
                    yaml.dump(export_data, f, default_flow_style=False, allow_unicode=True)

                messagebox.showinfo("Success", f"Exported to: {file_path}")

            except Exception as e:
                messagebox.showerror("Error", f"Export failed: {str(e)}")


def main():
    root = tk.Tk()
    app = MapCoordinateGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()