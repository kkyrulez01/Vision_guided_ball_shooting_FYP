from pathlib import Path
import cv2
import numpy as np
import json

from fyp_wamv_project.extract_hybrid_features import *
from fyp_wamv_project.RCE_neural_network import RCE_NN

def process_folder(folder_path):
    feature_vectors = []
    
    for file_path in folder_path.iterdir():
        # Load every sample
        sample = cv2.imread(str(file_path))

        # Extract hybrid features from each sample
        hybrid_vector, magnitude_spectrum, frequency_vectors = extract_hybrid_features(sample, U=32, V=32)
        feature_vectors.append(hybrid_vector) # Hybrid vector of shape (24,)
    
    return feature_vectors

def main():
    # Get path to templates
    template_dir = Path("src/fyp_wamv_project/fyp_wamv_project/templates")
    small_target_dir = template_dir / 'small_target' / 'resized'

    # Extract features for each class
    small_target_features = process_folder(small_target_dir)

    # Initialize the RCE Neural network
    model = RCE_NN(p_min=0.6)

    # Train the neural network
    model.cognize_entity(small_target_features, label="small_target")

    # Export weights to JSON
    export_data = {
        "p_min": model.p_min,
        "labels": model.labels,
        "prototypes": [p.tolist() for p in model.prototypes],
        "sigmas": [float(s) for s in model.std_devs],
    }

    output_file = Path('src/fyp_wamv_project/fyp_wamv_project/RCE_NN_weights/rce_weights.json')
    output_file.parent.mkdir(parents=True, exist_ok=True)

    with open(output_file, 'w') as f:
        json.dump(export_data, f, indent=4)

    print(f"Trained network weights saved to {output_file}")

if __name__ == 'main':
    main()
