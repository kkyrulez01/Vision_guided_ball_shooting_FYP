import numpy as np
import cv2

class RCE_NN():
    def __init__(self, p_min=0.7):

        self.p_min = p_min # Possibility threshold

        self.prototypes = [] # Mean feature vectors
        self.std_devs = [] # Standard deviation of the distance from feature vectors to mean vector
        self.labels = [] # Identity of entities

    def cognize_entity(self, feature_vectors, label):
        F = np.array(feature_vectors) # Shape of (k, 1028)
        k = len(F) # No. of sample images (patches) in this training session

        # Calculate mean vector 
        F_a = np.mean(F, axis=0) # Shape of (1028,)
        
        # Calculate standard deviation of distance from feature vectors to mean vector
        for j in range(k):
            squared_dist = np.dot((F[j] - F_a).T, (F[j] - F_a))
            total_squared_dist += squared_dist
        sigma = np.sqrt((1/k) * total_squared_dist)

        # Append trained node to middle layer
        self.prototypes.append(F_a)
        self.std_devs.append(sigma)
        self.labels.append(label)

        print(f"Cognized entity: {label}, sigma = {sigma:.4f}, ")
    
    