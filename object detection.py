import os
import cv2
import numpy as np
import time
import shutil
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
import threading
from queue import Queue
import glob

# Configuration
DRIVE_IMAGES_PATH = "/content/drive/MyDrive/robot_images"
DRIVE_RESULTS_PATH = "/content/drive/MyDrive/robot_results"
LOCAL_IMAGES_PATH = "/content/local_images"
LOCAL_RESULTS_PATH = "/content/local_results"
LOCAL_TEMP_PATH = "/content/temp_detection"  # New: temporary folder for 15-image collection
BATCH_SIZE = 5  # Process images in batches
CONFIDENCE_THRESHOLD = 0.5  # Minimum confidence for detection
MAX_WORKERS = 2  # Number of worker threads

# SPECIFY OBJECTS TO DETECT - Add the objects you want to detect here
TARGET_OBJECTS = ["dog", "bottle", "car"]  # Change this list to your desired objects
# Example: ["person", "car", "bicycle", "bottle", "cell phone", "laptop"]
# Leave empty [] to detect all objects

SAVE_MODE = "immediate"  # "immediate" or "interval"
SAVE_INTERVAL = 10  # seconds (only used if SAVE_MODE is "interval")
COLLECTION_SIZE = 15  # Number of images to collect before choosing best detection

# Create necessary directories
os.makedirs(LOCAL_IMAGES_PATH, exist_ok=True)
os.makedirs(LOCAL_RESULTS_PATH, exist_ok=True)
os.makedirs(LOCAL_TEMP_PATH, exist_ok=True)
os.makedirs(DRIVE_RESULTS_PATH, exist_ok=True)

class ObjectDetectionPipeline:
    def __init__(self):
        self.processed_images = set()  # Track processed images
        self.object_tracker = defaultdict(list)  # Track objects: {object_name: [(confidence, area, image_path, timestamp)]}
        self.image_queue = Queue()
        self.result_queue = Queue()
        self.running = True
        self.last_save_time = time.time()

        # New: Collection state tracking
        self.collecting_objects = {}  # {object_name: {'images': [], 'count': 0, 'started': False}}
        self.detected_objects_completed = set()  # Objects that have been fully processed
        self.current_target_objects = TARGET_OBJECTS.copy()  # Working copy of target objects

        # Colors for different object classes
        self.colors = np.random.uniform(0, 255, size=(len(class_names), 3))

        print(f"Target objects: {self.current_target_objects if self.current_target_objects else 'ALL OBJECTS'}")
        print(f"Collection size: {COLLECTION_SIZE} images per object")
        print(f"Save mode: {SAVE_MODE}")
        if SAVE_MODE == "interval":
            print(f"Save interval: {SAVE_INTERVAL} seconds")


    def get_timestamp_from_filename(self, filename):
      """Extract timestamp from filename HH_MM_SS_mm.jpg"""
      try:
          base_name = os.path.splitext(filename)[0]
          # Handle different filename formats
          if base_name.startswith("Kopia "):
              base_name = base_name[6:]  # Remove "Kopia " prefix

          # Try to extract timestamp parts
          parts = base_name.split('_')
          if len(parts) >= 3:
              # Take the last 3 or 4 parts as time components
              time_parts = parts[-4:] if len(parts) >= 4 else parts[-3:] + ['0']
              hour = int(time_parts[0])
              minute = int(time_parts[1])
              second = int(time_parts[2])
              millisecond = int(time_parts[3]) if len(time_parts) > 3 else 0

              # Create a timestamp for today with the extracted time
              import datetime
              today = datetime.date.today()
              dt = datetime.datetime.combine(today, datetime.time(hour, minute, second, millisecond * 10000))
              return dt.timestamp()

          return time.time()  # Fallback to current time
      except Exception as e:
          print(f"Timestamp parsing error for {filename}: {e}")
          return time.time()

    def copy_new_images_locally(self):
        """Copy new images from Drive to local storage for faster processing"""
        drive_images = glob.glob(os.path.join(DRIVE_IMAGES_PATH, "*.jpg"))
        new_images = []

        for drive_path in drive_images:
            filename = os.path.basename(drive_path)
            if filename not in self.processed_images:
                local_path = os.path.join(LOCAL_IMAGES_PATH, filename)
                try:
                    shutil.copy2(drive_path, local_path)
                    new_images.append(local_path)
                    self.processed_images.add(filename)
                except Exception as e:
                    print(f"Error copying {filename}: {e}")

        return new_images

    def save_image_to_temp(self, image_path, object_name):
        """Save image to temporary collection folder"""
        try:
            filename = os.path.basename(image_path)
            temp_filename = f"{object_name}_{filename}"
            temp_path = os.path.join(LOCAL_TEMP_PATH, temp_filename)
            shutil.copy2(image_path, temp_path)
            return temp_path
        except Exception as e:
            print(f"Error saving to temp: {e}")
            return None

    def clear_temp_collection(self, object_name):
        """Clear temporary collection for an object"""
        try:
            temp_files = glob.glob(os.path.join(LOCAL_TEMP_PATH, f"{object_name}_*"))
            for temp_file in temp_files:
                os.remove(temp_file)
            print(f"Cleared {len(temp_files)} temporary files for {object_name}")
        except Exception as e:
            print(f"Error clearing temp files for {object_name}: {e}")

    def start_collection_for_object(self, object_name):
        """Start collecting images for a specific object"""
        if object_name not in self.collecting_objects:
            self.collecting_objects[object_name] = {
                'images': [],
                'count': 0,
                'started': True
            }
            print(f"Started collecting images for: {object_name}")

    def add_to_collection(self, object_name, detection_data, image_path):
        """Add detection to collection for an object"""
        if object_name not in self.collecting_objects:
            self.start_collection_for_object(object_name)

        # Save image to temp folder
        temp_path = self.save_image_to_temp(image_path, object_name)
        if temp_path:
            detection_data['temp_path'] = temp_path
            self.collecting_objects[object_name]['images'].append(detection_data)
            self.collecting_objects[object_name]['count'] += 1

            print(f"Collected {self.collecting_objects[object_name]['count']}/{COLLECTION_SIZE} images for {object_name}")

    def finalize_collection(self, object_name):
        """Choose best detection from collection and save result"""
        if object_name not in self.collecting_objects:
            return False

        collection = self.collecting_objects[object_name]
        if not collection['images']:
            return False

        # Find detection with largest area
        best_detection = max(collection['images'], key=lambda x: x['area'])

        try:
            # Load the best image
            img = cv2.imread(best_detection['temp_path'])
            if img is None:
                print(f"Could not load best image for {object_name}")
                return False

            # Draw detection
            detection_info = {
                'label': object_name,
                'confidence': best_detection['confidence'],
                'bbox': best_detection['bbox'],
                'width_ratio': best_detection['width_ratio'],
                'height_ratio': best_detection['height_ratio']
            }

            annotated_img = self.draw_detections(img, [detection_info])

            # Create output filename with current timestamp
            #current_timestamp = time.strftime("%H_%M_%S_%f")[:-3]  # HH_MM_SS_mmm
            #output_filename = f"{object_name}_{current_timestamp}.jpg"
            # Find detection with largest area
            best_detection = max(collection['images'], key=lambda x: x['area'])

            # Use its original timestamp instead of now()
            frame_ts = best_detection['timestamp']
            formatted_ts = time.strftime("%H_%M_%S", time.localtime(frame_ts))
            # Remove the centiseconds part since your expected output is person_12_14_55.jpg
            output_filename = f"{object_name}_{formatted_ts}.jpg"


            # Save locally first (faster)
            local_output_path = os.path.join(LOCAL_RESULTS_PATH, output_filename)
            success = cv2.imwrite(local_output_path, annotated_img)

            if success:
                print(f"Saved best detection for {object_name}: {output_filename}")
                print(f"Best detection area: {best_detection['area']:.0f}, confidence: {best_detection['confidence']:.2f}")

                # Copy to Drive
                drive_path = os.path.join(DRIVE_RESULTS_PATH, output_filename)
                shutil.copy2(local_output_path, drive_path)
                print(f"Saved to Drive: {output_filename}")

                # Mark object as completed
                self.detected_objects_completed.add(object_name)

                # Remove from current target objects to prevent future detection
                if object_name in self.current_target_objects:
                    self.current_target_objects.remove(object_name)
                    print(f"Removed {object_name} from target objects list")

                return True
            else:
                print(f"Failed to save result for {object_name}")
                return False

        except Exception as e:
            print(f"Error finalizing collection for {object_name}: {e}")
            return False
        finally:
            # Clean up temporary files
            self.clear_temp_collection(object_name)
            # Remove from collecting objects
            del self.collecting_objects[object_name]

    def delete_processed_images(self, processed_paths):
        """Delete images after they've been processed (only if not in collection)"""
        for img_path in processed_paths:
            try:
                # Check if this image is part of any active collection
                filename = os.path.basename(img_path)
                is_in_collection = False

                for obj_name, collection in self.collecting_objects.items():
                    for detection in collection['images']:
                        if filename in detection.get('temp_path', ''):
                            is_in_collection = True
                            break
                    if is_in_collection:
                        break

                # Only delete if not in any collection
                if not is_in_collection:
                    # Delete local copy
                    if os.path.exists(img_path):
                        os.remove(img_path)

                    # Delete original from Drive
                    drive_path = os.path.join(DRIVE_IMAGES_PATH, filename)
                    if os.path.exists(drive_path):
                        os.remove(drive_path)
                        print(f"Deleted processed image: {filename}")

            except Exception as e:
                print(f"Error deleting {img_path}: {e}")


    def detect_objects_batch(self, image_paths):
    #"""Process a batch of images for object detection"""
      results = []
      successfully_processed = []

      for img_path in image_paths:
          try:
              # 1) Load image & run detection
              img = cv2.imread(img_path)
              if img is None:
                  continue
              detections, wr, hr = darknet_helper(img, width, height)

              filename  = os.path.basename(img_path)
              timestamp = self.get_timestamp_from_filename(filename)

              # 2) Build a list of valid detection_data dicts
              detection_data_list = []
              for label, confidence, bbox in detections:
                  confidence = float(confidence)
                  if confidence < CONFIDENCE_THRESHOLD:
                      continue
                  if label in self.detected_objects_completed:
                      continue
                  if self.current_target_objects and label not in self.current_target_objects:
                      continue

                  x,y,w,h = bbox
                  area = w * h
                  detection_data_list.append({
                      'label':        label,
                      'confidence':   confidence,
                      'bbox':         bbox,
                      'area':         area,
                      'width_ratio':  wr,
                      'height_ratio': hr,
                      'image_path':   img_path,
                      'timestamp':    timestamp
                  })

              # 3) Start collection on first detection of each object
              for dd in detection_data_list:
                  if dd['label'] not in self.collecting_objects:
                      # initialise with empty images list and frames counter
                      self.collecting_objects[dd['label']] = {
                          'images': [],       # will store detection_data dicts
                          'frames_seen': 0    # counts all frames since start
                      }
                      print(f"Started collecting frames for {dd['label']}")

              # 4) For every object currently collecting, save this frame & update state
              for label, state in list(self.collecting_objects.items()):
                  # save the raw frame for this object
                  temp_path = self.save_image_to_temp(img_path, label)
                  state['frames_seen'] += 1

                  # attach detection_data if present this frame
                  for dd in detection_data_list:
                      if dd['label'] == label:
                          dd['temp_path'] = temp_path
                          state['images'].append(dd)

                  # keep metadata for reporting
                  if dd['label'] == label:
                      detected = True

              # 5) Add to results list if any detections occurred
              if detection_data_list:
                  results.append({
                      'image_path': img_path,
                      'filename':   filename,
                      'detections': detection_data_list,
                      'timestamp':  timestamp
                  })

              successfully_processed.append(img_path)

          except Exception as e:
              print(f"Error processing {img_path}: {e}")

      # ─── After all frames, finalize any object whose frames_seen ≥ target ─────────
      for label, state in list(self.collecting_objects.items()):
          if state['frames_seen'] >= COLLECTION_SIZE:
              print(f"Collection complete for {label} ({state['frames_seen']} frames), finalizing...")
              self.finalize_collection(label)

      return results, successfully_processed


    def draw_detections(self, img, detections):
        """Draw bounding boxes and labels on image"""
        for detection in detections:
            label = detection['label']
            confidence = detection['confidence']
            bbox = detection['bbox']
            width_ratio = detection['width_ratio']
            height_ratio = detection['height_ratio']

            # Convert darknet bbox to opencv format
            x, y, w, h = bbox
            left = int((x - w/2) * width_ratio)
            top = int((y - h/2) * height_ratio)
            right = int((x + w/2) * width_ratio)
            bottom = int((y + h/2) * height_ratio)

            # Get color for this class
            color_idx = class_names.index(label) if label in class_names else 0
            color = self.colors[color_idx].tolist()

            # Draw bounding box
            cv2.rectangle(img, (left, top), (right, bottom), color, 2)

            # Draw label with confidence
            label_text = f"{label}: {confidence:.2f}"
            label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(img, (left, top - label_size[1] - 10),
                         (left + label_size[0], top), color, -1)
            cv2.putText(img, label_text, (left, top - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return img

    def background_result_saver(self):
        """Background thread to save results to Drive"""
        while self.running:
            try:
                if not self.result_queue.empty():
                    local_path, filename = self.result_queue.get(timeout=1)
                    drive_path = os.path.join(DRIVE_RESULTS_PATH, filename)
                    shutil.copy2(local_path, drive_path)
                    print(f"Saved result: {filename}")
                else:
                    time.sleep(0.1)
            except:
                continue

    def cleanup_old_files(self, max_files=100):
        """Clean up old local files to save space"""
        try:
            # Clean local images (but preserve those in active collections)
            local_files = glob.glob(os.path.join(LOCAL_IMAGES_PATH, "*.jpg"))
            if len(local_files) > max_files:
                local_files.sort(key=os.path.getctime)

                # Only remove files not in active collections
                for file_path in local_files[:-max_files]:
                    filename = os.path.basename(file_path)
                    is_in_collection = False

                    for obj_name, collection in self.collecting_objects.items():
                        for detection in collection['images']:
                            if filename in detection.get('image_path', ''):
                                is_in_collection = True
                                break
                        if is_in_collection:
                            break

                    if not is_in_collection:
                        os.remove(file_path)
                        self.processed_images.discard(filename)

            # Clean local results
            result_files = glob.glob(os.path.join(LOCAL_RESULTS_PATH, "*.jpg"))
            if len(result_files) > max_files:
                result_files.sort(key=os.path.getctime)
                for file_path in result_files[:-max_files]:
                    os.remove(file_path)

        except Exception as e:
            print(f"Cleanup error: {e}")

    def get_detection_status(self):
        """Get current detection status"""
        status = {
            'completed_objects': list(self.detected_objects_completed),
            'collecting_objects': {},
            'remaining_targets': self.current_target_objects.copy()
        }

        for obj_name, collection in self.collecting_objects.items():
            status['collecting_objects'][obj_name] = {
                'count': collection['frames_seen'],
                'target': COLLECTION_SIZE
            }

        return status

    def run(self):
        """Main processing loop"""
        print("Starting enhanced object detection pipeline...")
        print(f"Monitoring: {DRIVE_IMAGES_PATH}")
        print(f"Results: {DRIVE_RESULTS_PATH}")
        print(f"Target: >1 FPS processing rate")
        print("-" * 50)

        # Start background result saver
        result_thread = threading.Thread(target=self.background_result_saver)
        result_thread.daemon = True
        result_thread.start()

        frame_count = 0
        start_time = time.time()
        last_cleanup = time.time()
        last_status = time.time()

        try:
            while self.running:
                loop_start = time.time()

                # Copy new images locally
                new_images = self.copy_new_images_locally()

                if new_images:
                    print(f"Processing {len(new_images)} new images...")

                    # Process images in batches
                    all_processed_images = []
                    for i in range(0, len(new_images), BATCH_SIZE):
                        batch = new_images[i:i + BATCH_SIZE]

                        # Run detection on batch
                        batch_start = time.time()
                        detection_results, successfully_processed = self.detect_objects_batch(batch)
                        batch_time = time.time() - batch_start

                        # Collect successfully processed images for deletion
                        all_processed_images.extend(successfully_processed)

                        # Print detection summary
                        if detection_results:
                            total_detections = sum(len(r['detections']) for r in detection_results)
                            unique_objects = set()
                            for r in detection_results:
                                for d in r['detections']:
                                    unique_objects.add(d['label'])

                            print(f"Batch: {len(batch)} images, {total_detections} detections, "
                                  f"{len(unique_objects)} unique objects, {batch_time:.2f}s")
                        else:
                            print(f"Batch: {len(batch)} images, no target objects detected")

                    # Delete processed images (respecting collection requirements)
                    if all_processed_images:
                        self.delete_processed_images(all_processed_images)

                    frame_count += len(new_images)

                # Print status every 30 seconds
                if time.time() - last_status > 30:
                    status = self.get_detection_status()
                    print(f"Status - Completed: {status['completed_objects']}")
                    print(f"Status - Collecting: {status['collecting_objects']}")
                    print(f"Status - Remaining targets: {status['remaining_targets']}")
                    last_status = time.time()

                # Calculate and display FPS
                elapsed = time.time() - start_time
                if elapsed > 0:
                    fps = frame_count / elapsed
                    print(f"Average FPS: {fps:.2f} | Processed: {frame_count} images")

                # Periodic cleanup
                if time.time() - last_cleanup > 60:  # Every minute
                    self.cleanup_old_files()
                    last_cleanup = time.time()

                # Maintain target FPS
                loop_time = time.time() - loop_start
                target_loop_time = 1.0  # 1 second for 1 FPS minimum
                if loop_time < target_loop_time:
                    time.sleep(target_loop_time - loop_time)

        except KeyboardInterrupt:
            print("\nStopping pipeline...")
            self.running = False

            # Finalize any incomplete collections
            print("Finalizing incomplete collections...")
            for obj_name in list(self.collecting_objects.keys()):
                if self.collecting_objects[obj_name]['count'] > 0:
                    print(f"Finalizing incomplete collection for {obj_name} ({self.collecting_objects[obj_name]['count']} images)")
                    self.finalize_collection(obj_name)

# Create and run the pipeline
pipeline = ObjectDetectionPipeline()

# Start the detection pipeline
print("Starting enhanced real-time object detection...")
print("Press Ctrl+C to stop")
pipeline.run()