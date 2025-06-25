   elo def detect_objects_batch(self, image_paths):
        """Process a batch of images for object detection"""
        results = []
        successfully_processed = []

        for img_path in image_paths:
            try:
                # Load image
                img = cv2.imread(img_path)
                if img is None:
                    continue

                # Run detection
                detections, width_ratio, height_ratio = darknet_helper(img, width, height)

                # Process detections
                filename = os.path.basename(img_path)
                timestamp = self.get_timestamp_from_filename(filename)

                detected_objects = []
                for detection in detections:
                    label, confidence, bbox = detection

                    # Convert confidence to float if it's a string
                    try:
                        confidence = float(confidence)
                    except (ValueError, TypeError):
                        confidence = 0.0

                    # Filter by target objects if specified
                    if TARGET_OBJECTS and label not in TARGET_OBJECTS:
                        continue

                    if confidence > CONFIDENCE_THRESHOLD:
                        # Calculate bounding box area
                        x, y, w, h = bbox
                        area = w * h
                        detected_objects.append({
                            'label': label,
                            'confidence': confidence,
                            'bbox': bbox,
                            'area': area,
                            'width_ratio': width_ratio,
                            'height_ratio': height_ratio
                        })

                        # Track this object
                        self.object_tracker[label].append({
                            'confidence': confidence,
                            'area': area,
                            'image_path': img_path,
                            'timestamp': timestamp,
                            'bbox': bbox,
                            'width_ratio': width_ratio,
                            'height_ratio': height_ratio
                        })

                if detected_objects:
                    results.append({
                        'image_path': img_path,
                        'filename': filename,
                        'detections': detected_objects,
                        'timestamp': timestamp
                    })

                # Mark as successfully processed
                successfully_processed.append(img_path)

            except Exception as e:
                print(f"Error processing {img_path}: {e}")

        return results, successfully_processed