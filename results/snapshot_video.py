import cv2
import os

def capture_snapshots(input_file, output_dir, num_snapshots):
    # Open the video file
    cap = cv2.VideoCapture(input_file)

    if not cap.isOpened():
        print("Error: Could not open video file.")
        return

    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_interval = total_frames // num_snapshots

    for i in range(num_snapshots):
        # Set the frame position to capture
        frame_pos = i * frame_interval
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_pos)

        # Read the frame
        ret, frame = cap.read()
        if not ret:
            print(f"Error: Unable to read frame {i}")
            break

        # Save the frame as an image
        output_path = os.path.join(output_dir, f'snapshot_{i+1}.jpg')
        cv2.imwrite(output_path, frame)
        print(f"Saved snapshot {i+1} at {output_path}")

    # Release the video capture object
    cap.release()

if __name__ == "__main__":
    input_file = "platoon_dynamic_obstacle.mp4"  # Replace with your input video file path
    output_dir = "snapshots"  # Replace with the directory where you want to save snapshots
    num_snapshots = 10  # Replace with the number of snapshots you want

    capture_snapshots(input_file, output_dir, num_snapshots)
