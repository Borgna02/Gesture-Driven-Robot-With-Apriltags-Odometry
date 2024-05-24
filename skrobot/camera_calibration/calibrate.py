import numpy as np
import cv2
import glob

# Definisci le dimensioni della scacchiera
chessboard_size = (9, 7)  # Numero di angoli interni della scacchiera

# Prepara gli array per i punti del mondo reale e i punti dell'immagine
objpoints = []  # Punti 3D del mondo reale
imgpoints = []  # Punti 2D dell'immagine

# Genera i punti del mondo reale
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Carica le immagini della scacchiera
images = glob.glob('image*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Trova gli angoli della scacchiera
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Disegna e mostra gli angoli sulla scacchiera
        img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard Corners', img)
        cv2.imwrite("calib/calib_" + fname, img)
        cv2.waitKey(500)

# Calibra la camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Salva i parametri di calibrazione
np.save('camera_matrix.npy', mtx)
np.save('distortion_coefficients.npy', dist)

# Mostra i risultati della calibrazione
print("Matrice della camera:\n", mtx)
print("Coefficienti di distorsione:\n", dist)

cv2.destroyAllWindows()