import cv2
import numpy as np
import matplotlib.pyplot as plt

from fyp_wamv_project.image_sampling import *

def extract_spatial_features(image_patch, U=32, V=32):
    """
    Extract features in the time/space-domain by calculating the mean value of: 
    1) approximate electromagnetic energy
    2) Square root of variance of approximate electromagnetic energy
    3) Horizontal distribution of approximate electromagnetic energy
    4) Vertical distribution of approximate electromagnetic energy

    Returns a spatial vector that represents the image patch.
    Spatial vector, F = [I_a, sig_I, sig_u, sig_v]
    """
    normalized_patch = cv2.resize(image_patch, (U,V), interpolation=cv2.INTER_AREA)

    # Convert to gray scale
    if len(normalized_patch.shape) == 3: # if 3 color channels
        I = cv2.cvtColor(normalized_patch, cv2.COLOR_BGR2GRAY)

    else: # if already gray scale
        I = normalized_patch
    
    # Extract features in Time/Spatial-Domain
    # Create a U x V mesh grid with coordinates
    u = np.arange(U)
    v = np.arange(V)
    uu, vv = np.meshgrid(u,v)

    # Mean value I_a of approximate electromagnetic energy (Intensity)
    I_a = np.mean(I)

    # Square root of the variance (standard deviation) sig_I of approximate Electromagnetic energy
    sig_I = np.std(I)

    sum_I = np.sum(I)
    if sum_I == 0:
        spatial_vector = np.array([0,0,0,0], dtype=np.float32)
    else:
        # Centre of masses
        uc = np.sum(I * uu) / sum_I
        vc = np.sum(I * vv) / sum_I

        # Horizontal distribution (sig_u) of approximate electromagnetic energy
        sig_u = np.sqrt(np.sum(I * ((uu - uc) ** 2)) / sum_I)
        # Vertical distribution (sig_v) of approximate electromagnetic energy
        sig_v = np.sqrt(np.sum(I * ((vv - vc) ** 2)) / sum_I)

        spatial_vector = np.array([I_a, sig_I, sig_u, sig_v])

        # Normalize the spatial vector
        spatial_vector[0] /= 255 # Max intensity is 255
        spatial_vector[1] /= 128.0 # approx max std dev
        spatial_vector[2] /= float(U)
        spatial_vector[3] /= float(V)

    return spatial_vector

def extract_frequency_features(image_patch, U=32, V=32):
    """
    Extract features in frequency domain using FFT.

    Returns a frequency vector that represents the image patch. 
    """
    normalized_patch = cv2.resize(image_patch, (U,V), interpolation=cv2.INTER_AREA)
    # Convert to gray scale
    if len(normalized_patch.shape) == 3: # if 3 color channels
        I = cv2.cvtColor(normalized_patch, cv2.COLOR_BGR2GRAY)

    else: # if already gray scale
        I = normalized_patch.astype(float32)
    
    # Computes the two-dimensional discrete fourier transform (DFT) using the FFT algorithm
    f_transform = np.fft.fft2(I)
    # By default, lower frequency values are in the corners of the matrix
    # This shifts the lower frequency values to the center instead, and the higher frequencies to the edges
    f_shift = np.fft.fftshift(f_transform)

    # Calculate the maximum magnitude of each frequency to obtain a magnitude spectrum
    magnitude_spectrum = np.float32(20 * np.log(np.abs(f_shift) + 1))

    # Normalize magnitude_spectrum to 0-1 range
    magnitude_spectrum = cv2.normalize(magnitude_spectrum, None, 0, 1, cv2.NORM_MINMAX)

    # Flatten into a 1D vector
    frequency_vector = magnitude_spectrum.flatten()

    return frequency_vector, magnitude_spectrum

def extract_hybrid_features(image_patch, U=32, V=32):
    """
    Combines feature extraction in time-domain and frequency domain and
    returns a hybrid feature vector.
    """
    normalized_patch = cv2.resize(image_patch, (U,V), interpolation=cv2.INTER_AREA)

    # Convert to gray scale
    if len(normalized_patch.shape) == 3: # if 3 color channels
        I = cv2.cvtColor(normalized_patch, cv2.COLOR_BGR2GRAY)

    else: # if already gray scale
        I = normalized_patch.astype(float32)
    
    spatial_vector = extract_spatial_features(image_patch, U=32, V=32)
    frequency_vector, magnitude_spectrum = extract_frequency_features(image_patch, U=32, V=32)
    hybrid_vector = np.concatenate((spatial_vector, frequency_vector))

    return hybrid_vector, magnitude_spectrum

def main():
    # Example usage
    image = cv2.imread('src/fyp_wamv_project/fyp_wamv_project/example_images/left_camera_feed.png')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # Slice the image for testing
    patches = slice_image_to_grid(image, d_v=3, d_h=3)
    
    # Extract hybrid feature from each patch
    all_hybrid_signatures = []
    all_magnitude_spectrums = []

    for patch in patches:
        hybrid_signature, magnitude_spectrum = extract_hybrid_features(image_patch=patch)
        all_hybrid_signatures.append(hybrid_signature)
        all_magnitude_spectrums.append(magnitude_spectrum)

    print(f"Hybrid Vector length: {len(hybrid_signature)}")
    
    # Plot example patch and its feature vector
    fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(15,10))

    for i in range(axes.shape[0]):
        # Plot patches in first column
        axes[i,0].imshow(patches[i])
        # Plot intensity maps in second column
        axes[i,1].imshow(all_magnitude_spectrums[i], cmap='gray')

    plt.show()
if __name__ == 'main':
    main()