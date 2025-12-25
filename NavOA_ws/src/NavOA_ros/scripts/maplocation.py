#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import os
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
from PIL import Image, ImageTk
import math
from datetime import datetime
import subprocess
import threading
import time


class AutoNavGoalPublisher:
    def __init__(self, root):
        self.root = root
        self.root.title("Auto Nav Goal Publisher")
        self.root.geometry("900x500")
        
        # Map data
        self.map_image = None
        self.map_data = None
        self.current_pose = None
        self.mouse_pressed = False
        self.drag_start_pos = None
        self.drag_current_pos = None
        
        # Points history
        self.points = []
        self.publish_count = 0
        
        # ROS environment (modify if needed)
        self.ros_env = os.environ.copy()
        self.ros_env['ROS_MASTER_URI'] = 'http://localhost:11311'
        self.ros_env['ROS_HOSTNAME'] = 'localhost'
        
        # Create interface
        self.create_widgets()

    def create_widgets(self):
        # Main container
        main_paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, sashrelief=tk.RAISED, sashwidth=4)
        main_paned.pack(fill=tk.BOTH, expand=True)
        
        # Left panel - Map display
        left_frame = tk.Frame(main_paned, bg="#1e1e1e")
        main_paned.add(left_frame, minsize=500)
        
        # Right panel - Controls and terminal
        right_frame = tk.Frame(main_paned, bg="#2b2b2b")
        main_paned.add(right_frame, minsize=400)
        
        # Setup left panel
        self.setup_map_panel(left_frame)
        
        # Setup right panel
        self.setup_control_panel(right_frame)

    def setup_map_panel(self, parent):
        """Setup map display panel"""
        # Top control bar
        control_bar = tk.Frame(parent, bg="#333333", height=40)
        control_bar.pack(fill=tk.X, pady=(0, 2))
        
        # Load buttons
        btn_frame = tk.Frame(control_bar, bg="#333333")
        btn_frame.pack(side=tk.LEFT, padx=10)
        
        load_yaml_btn = tk.Button(btn_frame, text="Load Map",
                                  command=self.load_yaml_file,
                                  width=10, bg="#4CAF50", fg="white")
        load_yaml_btn.pack(side=tk.LEFT, padx=2)
        
        clear_btn = tk.Button(btn_frame, text="Clear Points",
                              command=self.clear_points,
                              width=10, bg="#f44336", fg="white")
        clear_btn.pack(side=tk.LEFT, padx=2)
        
        # Status label
        self.status_label = tk.Label(control_bar, text="Load map and click to set goal",
                                     bg="#333333", fg="white", font=("Arial", 10))
        self.status_label.pack(side=tk.RIGHT, padx=10)
        
        # Map canvas
        self.canvas = tk.Canvas(parent, bg="#1e1e1e", cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Coordinate display
        self.coord_label = tk.Label(parent, text="Coord: (0.000, 0.000)",
                                   bg="#333333", fg="#4CAF50", font=("Arial", 10))
        self.coord_label.pack(fill=tk.X, side=tk.BOTTOM)
        
        # Bind mouse events
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_release)
        self.canvas.bind("<Motion>", self.on_mouse_move)

    def setup_control_panel(self, parent):
        """Setup control and terminal panel"""
        # Control buttons frame
        btn_frame = tk.Frame(parent, bg="#2b2b2b")
        btn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Publish current button
        self.publish_btn = tk.Button(btn_frame, text="PUBLISH GOAL",
                                     command=self.publish_current_goal,
                                     width=15, height=2, 
                                     bg="#2196F3", fg="white",
                                     font=("Arial", 11, "bold"),
                                     state=tk.DISABLED)
        self.publish_btn.pack(side=tk.LEFT, padx=5)
        
        # Publish all button
        self.publish_all_btn = tk.Button(btn_frame, text="PUBLISH ALL",
                                         command=self.publish_all_goals,
                                         width=12, height=2,
                                         bg="#4CAF50", fg="white",
                                         font=("Arial", 11),
                                         state=tk.DISABLED)
        self.publish_all_btn.pack(side=tk.LEFT, padx=5)
        
        # Stop button
        self.stop_btn = tk.Button(btn_frame, text="STOP",
                                  command=self.stop_publishing,
                                  width=8, height=2,
                                  bg="#f44336", fg="white",
                                  font=("Arial", 11),
                                  state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Stats frame
        stats_frame = tk.Frame(parent, bg="#2b2b2b")
        stats_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.stats_label = tk.Label(stats_frame, 
                                   text="Published: 0 goals",
                                   bg="#2b2b2b", fg="#00bcd4",
                                   font=("Arial", 10))
        self.stats_label.pack()
        
        # Current goal info
        info_frame = tk.LabelFrame(parent, text="Current Goal",
                                   font=("Arial", 11, "bold"),
                                   bg="#2b2b2b", fg="white")
        info_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.goal_info = tk.Text(info_frame, height=4, width=40,
                                 bg="#3c3c3c", fg="white",
                                 font=("Courier", 9))
        self.goal_info.pack(padx=5, pady=5)
        
        # Terminal output
        term_frame = tk.LabelFrame(parent, text="Terminal Output",
                                   font=("Arial", 11, "bold"),
                                   bg="#2b2b2b", fg="white")
        term_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.terminal_output = scrolledtext.ScrolledText(term_frame, height=10,
                                                         bg="#1e1e1e", fg="#00ff00",
                                                         font=("Courier", 9))
        self.terminal_output.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add clear button for terminal
        clear_term_btn = tk.Button(term_frame, text="Clear Output",
                                   command=self.clear_terminal,
                                   width=12, bg="#ff9800", fg="white")
        clear_term_btn.pack(pady=5)

    def log_terminal(self, message):
        """Add message to terminal output"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.terminal_output.insert(tk.END, f"[{timestamp}] {message}\n")
        self.terminal_output.see(tk.END)
        self.terminal_output.update()

    def clear_terminal(self):
        """Clear terminal output"""
        self.terminal_output.delete(1.0, tk.END)

    def update_stats(self):
        """Update statistics"""
        self.stats_label.config(text=f"Published: {self.publish_count} goals")

    def update_goal_info(self, pose_data):
        """Update current goal information"""
        self.goal_info.delete(1.0, tk.END)
        
        info = f"Position: ({pose_data['world'][0]:.3f}, {pose_data['world'][1]:.3f})\n"
        info += f"Angle: {pose_data['angle']:.1f} deg\n"
        info += f"Quaternion: z={pose_data['quaternion'][2]:.6f}, w={pose_data['quaternion'][3]:.6f}"
        
        self.goal_info.insert(1.0, info)

    def load_yaml_file(self):
        """Load YAML configuration file"""
        file_path = filedialog.askopenfilename(
            title="Select Map YAML File",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )

        if file_path:
            try:
                with open(file_path, 'r') as f:
                    self.map_data = yaml.safe_load(f)

                # Auto load corresponding image
                map_dir = os.path.dirname(file_path)
                image_file = os.path.join(map_dir, self.map_data['image'])

                if os.path.exists(image_file):
                    self.load_map_image(image_file)
                    self.log_terminal(f"Map loaded: {os.path.basename(file_path)}")
                    self.status_label.config(text="Click on map to set goal")
                else:
                    messagebox.showwarning("Warning", f"Image file not found: {image_file}")

            except Exception as e:
                messagebox.showerror("Error", f"Failed to load YAML file: {str(e)}")
                self.log_terminal(f"Error: {str(e)}")

    def load_map_image(self, image_path):
        """Load and display map image"""
        try:
            # Use PIL to load image
            pil_image = Image.open(image_path)

            # Convert to RGB if needed
            if pil_image.mode in ('RGBA', 'LA'):
                rgb_image = Image.new('RGB', pil_image.size, (255, 255, 255))
                rgb_image.paste(pil_image, mask=pil_image.split()[-1])
                pil_image = rgb_image

            # Adjust image size
            self.original_image = pil_image.copy()
            self.display_image = self.resize_image(pil_image, max_width=480, max_height=400)

            # Convert to Tkinter PhotoImage
            self.tk_image = ImageTk.PhotoImage(self.display_image)

            # Display on canvas
            self.canvas.delete("all")
            self.canvas.config(width=self.display_image.width, height=self.display_image.height)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image, tags="map")

            self.log_terminal(f"Image loaded: {os.path.basename(image_path)}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to load image: {str(e)}")
            self.log_terminal(f"Error: {str(e)}")

    def resize_image(self, image, max_width=480, max_height=400):
        """Resize image"""
        width, height = image.size
        width_ratio = max_width / width
        height_ratio = max_height / height
        ratio = min(width_ratio, height_ratio)

        if ratio < 1:
            new_width = int(width * ratio)
            new_height = int(height * ratio)
            return image.resize((new_width, new_height), Image.Resampling.LANCZOS)

        return image

    def pixel_to_world(self, pixel_x, pixel_y):
        """Convert pixel coordinates to world coordinates"""
        if not self.map_data or not hasattr(self, 'original_image'):
            return None

        original_width, original_height = self.original_image.size
        resolution = self.map_data['resolution']
        origin_x = self.map_data['origin'][0]
        origin_y = self.map_data['origin'][1]

        world_x = origin_x + pixel_x * resolution
        world_y = origin_y + (original_height - pixel_y) * resolution

        return world_x, world_y

    def canvas_to_pixel(self, canvas_x, canvas_y):
        """Convert canvas coordinates to original pixel coordinates"""
        if not hasattr(self, 'original_image'):
            return None

        display_width = self.display_image.width
        display_height = self.display_image.height
        original_width, original_height = self.original_image.size

        scale_x = original_width / display_width
        scale_y = original_height / display_height

        original_pixel_x = int(canvas_x * scale_x)
        original_pixel_y = int(canvas_y * scale_y)

        return original_pixel_x, original_pixel_y

    def on_canvas_click(self, event):
        """Handle canvas left click event"""
        if not self.map_data or not hasattr(self, 'original_image'):
            return

        self.mouse_pressed = True
        self.drag_start_pos = (event.x, event.y)
        self.drag_current_pos = (event.x, event.y)

        # Get pixel coordinates
        pixel_x, pixel_y = self.canvas_to_pixel(event.x, event.y)
        world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)

        # Clear previous temporary graphics
        self.clear_temp_graphics()

        # Set current position
        self.current_pose = {
            'pixel': (pixel_x, pixel_y),
            'world': (world_x, world_y),
            'canvas': (event.x, event.y),
            'angle': 0.0,
            'quaternion': (0.0, 0.0, 0.0, 1.0)
        }

        # Draw initial position marker
        self.draw_position_marker(event.x, event.y)

        self.status_label.config(
            text=f"Set position: ({world_x:.2f}, {world_y:.2f}) - Drag for direction")
        
        # Enable publish button
        self.publish_btn.config(state=tk.NORMAL)

    def on_mouse_drag(self, event):
        """Handle mouse drag event"""
        if self.mouse_pressed and self.current_pose:
            self.drag_current_pos = (event.x, event.y)

            # Calculate direction angle
            dx = event.x - self.current_pose['canvas'][0]
            dy = event.y - self.current_pose['canvas'][1]

            # Calculate angle
            angle_rad = math.atan2(dy, dx)
            angle_deg = math.degrees(angle_rad)

            # Update current angle
            self.current_pose['angle'] = angle_deg

            # Calculate quaternion
            w = math.cos(angle_rad / 2)
            z = math.sin(angle_rad / 2)
            self.current_pose['quaternion'] = (0.0, 0.0, z, w)

            # Clear and redraw
            self.clear_temp_graphics()
            self.draw_position_marker(self.current_pose['canvas'][0],
                                      self.current_pose['canvas'][1])
            self.draw_direction_arrow(self.current_pose['canvas'],
                                      self.drag_current_pos,
                                      angle_deg)

            self.status_label.config(
                text=f"Direction: {angle_deg:.0f} deg - Release to confirm")

    def on_mouse_release(self, event):
        """Handle mouse release event"""
        if self.mouse_pressed and self.current_pose:
            self.mouse_pressed = False

            if self.drag_start_pos and self.drag_current_pos:
                dx = self.drag_current_pos[0] - self.current_pose['canvas'][0]
                dy = self.drag_current_pos[1] - self.current_pose['canvas'][1]

                # Calculate final angle
                angle_rad = math.atan2(dy, dx)
                angle_deg = math.degrees(angle_rad)

                # Update angle and quaternion
                self.current_pose['angle'] = angle_deg
                w = math.cos(angle_rad / 2)
                z = math.sin(angle_rad / 2)
                self.current_pose['quaternion'] = (0.0, 0.0, z, w)

                # Save to history
                self.save_current_pose()

                # Update goal info
                self.update_goal_info(self.current_pose)

                self.status_label.config(
                    text=f"Goal ready: ({self.current_pose['world'][0]:.2f}, "
                         f"{self.current_pose['world'][1]:.2f}), {angle_deg:.0f} deg")

                # Enable publish all button if we have points
                if self.points:
                    self.publish_all_btn.config(state=tk.NORMAL)

            self.drag_start_pos = None
            self.drag_current_pos = None

    def on_mouse_move(self, event):
        """Handle mouse move event"""
        if not self.map_data or not hasattr(self, 'original_image'):
            return

        # Get pixel coordinates
        pixel_coords = self.canvas_to_pixel(event.x, event.y)
        if pixel_coords:
            pixel_x, pixel_y = pixel_coords
            world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)

            angle_text = ""
            if self.current_pose and self.drag_current_pos:
                dx = event.x - self.current_pose['canvas'][0]
                dy = event.y - self.current_pose['canvas'][1]
                angle_rad = math.atan2(dy, dx)
                angle_deg = math.degrees(angle_rad)
                angle_text = f", {angle_deg:.0f} deg"

            self.coord_label.config(
                text=f"Coord: ({world_x:.2f}, {world_y:.2f}){angle_text}")

    def clear_temp_graphics(self):
        """Clear temporary graphics"""
        for item in self.canvas.find_withtag("temp"):
            self.canvas.delete(item)

    def draw_position_marker(self, x, y):
        """Draw position marker"""
        self.canvas.create_oval(
            x - 4, y - 4, x + 4, y + 4,
            fill='red', outline='white', width=2, tags="temp"
        )
        
        self.canvas.create_line(x - 8, y, x - 2, y,
                                fill='white', width=1, tags="temp")
        self.canvas.create_line(x + 2, y, x + 8, y,
                                fill='white', width=1, tags="temp")
        self.canvas.create_line(x, y - 8, x, y - 2,
                                fill='white', width=1, tags="temp")
        self.canvas.create_line(x, y + 2, x, y + 8,
                                fill='white', width=1, tags="temp")

    def draw_direction_arrow(self, start, end, angle):
        """Draw direction arrow"""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx * dx + dy * dy)

        min_length = 15
        if length < min_length:
            scale = min_length / length if length > 0 else 1
            end = (start[0] + dx * scale, start[1] + dy * scale)

        self.canvas.create_line(start[0], start[1], end[0], end[1],
                                fill='#00ff00', width=2, arrow=tk.LAST,
                                arrowshape=(10, 12, 5), tags="temp")
        
        self.canvas.create_text(start[0] + 25, start[1] - 20,
                                text=f"{angle:.0f} deg",
                                fill='#ff9800', font=('Arial', 9, 'bold'),
                                tags="temp")

    def save_current_pose(self):
        """Save current pose to history"""
        if not self.current_pose:
            return

        # Add timestamp
        self.current_pose['timestamp'] = datetime.now().strftime("%H:%M:%S")
        
        # Add to history list
        self.points.append(self.current_pose.copy())
        
        # Draw permanent marker
        self.draw_permanent_marker(len(self.points), 
                                  self.current_pose['canvas'],
                                  self.current_pose['angle'])

    def draw_permanent_marker(self, idx, canvas_pos, angle):
        """Draw permanent marker"""
        x, y = canvas_pos

        # Draw position point
        self.canvas.create_oval(
            x - 5, y - 5, x + 5, y + 5,
            fill='#2196F3', outline='white', width=2,
            tags=f"permanent_{idx}"
        )

        # Draw direction arrow
        arrow_length = 20
        angle_rad = math.radians(angle)
        end_x = x + arrow_length * math.cos(angle_rad)
        end_y = y + arrow_length * math.sin(angle_rad)

        self.canvas.create_line(x, y, end_x, end_y,
                                fill='#4CAF50', width=1, arrow=tk.LAST,
                                arrowshape=(8, 10, 4),
                                tags=f"permanent_{idx}")

        # Draw index number
        self.canvas.create_text(x, y - 15,
                                text=str(idx),
                                fill='yellow', font=('Arial', 9),
                                tags=f"permanent_{idx}")

    def create_ros_command(self, pose_data):
        """Create ROS terminal command"""
        x, y = pose_data['world']
        qz, qw = pose_data['quaternion'][2], pose_data['quaternion'][3]
        
        cmd = f"rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "
        cmd += f"'{{header: {{frame_id: \"map\"}}, "
        cmd += f"pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: 0.0}}, "
        cmd += f"orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}}}}}'"
        
        return cmd

    def execute_command(self, cmd):
        """Execute ROS command in terminal"""
        try:
            self.log_terminal(f"Executing: {cmd}")
            
            # Execute the command
            process = subprocess.Popen(
                ['bash', '-c', f'source /opt/ros/noetic/setup.bash && {cmd}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.ros_env,
                text=True
            )
            
            # Read output in real-time
            stdout, stderr = process.communicate(timeout=5)
            
            if stdout:
                self.log_terminal(f"Output: {stdout.strip()}")
            if stderr:
                self.log_terminal(f"Error: {stderr.strip()}")
                
            if process.returncode == 0:
                self.log_terminal("Command executed successfully!")
                return True
            else:
                self.log_terminal(f"Command failed with code: {process.returncode}")
                return False
                
        except subprocess.TimeoutExpired:
            self.log_terminal("Command timed out!")
            process.kill()
            return False
        except Exception as e:
            self.log_terminal(f"Error executing command: {str(e)}")
            return False

    def publish_current_goal(self):
        """Publish the current goal"""
        if not self.current_pose:
            self.log_terminal("No goal set to publish!")
            return
            
        # Disable buttons during publishing
        self.publish_btn.config(state=tk.DISABLED, text="PUBLISHING...")
        self.stop_btn.config(state=tk.NORMAL)
        
        # Create and execute command
        cmd = self.create_ros_command(self.current_pose)
        
        def publish_thread():
            success = self.execute_command(cmd)
            
            # Update UI in main thread
            self.root.after(0, lambda: self.on_publish_complete(success))
            
            # Re-enable buttons
            self.root.after(0, lambda: self.publish_btn.config(
                state=tk.NORMAL, text="PUBLISH GOAL"))
            self.root.after(0, lambda: self.stop_btn.config(state=tk.DISABLED))
        
        # Start publishing in background thread
        thread = threading.Thread(target=publish_thread, daemon=True)
        thread.start()

    def publish_all_goals(self):
        """Publish all goals in sequence"""
        if not self.points:
            self.log_terminal("No goals to publish!")
            return
            
        self.log_terminal(f"Starting to publish {len(self.points)} goals...")
        
        # Disable buttons during publishing
        self.publish_all_btn.config(state=tk.DISABLED, text="PUBLISHING...")
        self.stop_btn.config(state=tk.NORMAL)
        
        def publish_all_thread():
            for i, point in enumerate(self.points, 1):
                if hasattr(self, '_stop_publishing') and self._stop_publishing:
                    self.log_terminal("Publishing stopped!")
                    break
                    
                self.root.after(0, lambda idx=i: 
                    self.status_label.config(text=f"Publishing goal {idx}/{len(self.points)}"))
                
                cmd = self.create_ros_command(point)
                self.log_terminal(f"Publishing goal {i}: ({point['world'][0]:.2f}, {point['world'][1]:.2f})")
                
                success = self.execute_command(cmd)
                
                if success:
                    self.publish_count += 1
                    self.root.after(0, self.update_stats)
                
                # Wait between goals (except for the last one)
                if i < len(self.points) and not (hasattr(self, '_stop_publishing') and self._stop_publishing):
                    self.log_terminal(f"Waiting 2 seconds before next goal...")
                    time.sleep(2)
            
            # Clean up
            if hasattr(self, '_stop_publishing'):
                del self._stop_publishing
                
            # Update UI in main thread
            self.root.after(0, lambda: self.on_publish_all_complete())
            
            # Re-enable buttons
            self.root.after(0, lambda: self.publish_all_btn.config(
                state=tk.NORMAL, text="PUBLISH ALL"))
            self.root.after(0, lambda: self.stop_btn.config(state=tk.DISABLED))
            self.root.after(0, lambda: self.status_label.config(
                text=f"Published {self.publish_count} goals"))
        
        # Start publishing in background thread
        thread = threading.Thread(target=publish_all_thread, daemon=True)
        thread.start()

    def stop_publishing(self):
        """Stop publishing process"""
        self._stop_publishing = True
        self.log_terminal("Stopping publish process...")
        self.stop_btn.config(state=tk.DISABLED, text="STOPPING...")

    def on_publish_complete(self, success):
        """Handle single publish completion"""
        if success:
            self.publish_count += 1
            self.update_stats()
            self.log_terminal(f"Goal published successfully! Total: {self.publish_count}")

    def on_publish_all_complete(self):
        """Handle all publish completion"""
        self.log_terminal(f"All goals published! Total: {self.publish_count}")

    def clear_points(self):
        """Clear all points"""
        self.points = []
        self.current_pose = None
        
        # Clear canvas markers
        for item in self.canvas.find_all():
            tags = self.canvas.gettags(item)
            if any(tag.startswith('permanent_') for tag in tags):
                self.canvas.delete(item)
        
        # Clear temporary graphics
        self.clear_temp_graphics()
        
        # Clear goal info
        self.goal_info.delete(1.0, tk.END)
        
        # Disable buttons
        self.publish_btn.config(state=tk.DISABLED)
        self.publish_all_btn.config(state=tk.DISABLED)
        
        self.status_label.config(text="Points cleared")
        self.log_terminal("All points cleared")


def main():
    root = tk.Tk()
    app = AutoNavGoalPublisher(root)
    root.mainloop()


if __name__ == "__main__":
    main()