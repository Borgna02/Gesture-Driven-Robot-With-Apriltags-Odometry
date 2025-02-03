# import numpy as np
# import cv2
# import glob

# # Definisci le dimensioni della scacchiera
# chessboard_size = (9, 7)  # Numero di angoli interni della scacchiera

# # Prepara gli array per i punti del mondo reale e i punti dell'immagine
# objpoints = []  # Punti 3D del mondo reale
# imgpoints = []  # Punti 2D dell'immagine

# # Genera i punti del mondo reale
# objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
# objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# # Carica le immagini della scacchiera
# images = glob.glob('image*.png')

# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     # Trova gli angoli della scacchiera
#     ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

#     if ret:
#         objpoints.append(objp)
#         imgpoints.append(corners)

#         # Disegna e mostra gli angoli sulla scacchiera
#         img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
#         # cv2.imshow('Chessboard Corners', img)
#         # cv2.imwrite("calib/calib_" + fname, img)
#         cv2.waitKey(500)

# # Imposta il formato di stampa per evitare la notazione scientifica
# np.set_printoptions(suppress=True)

# # Calibra la camera
# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# # Salva i parametri di calibrazione
# np.save('camera_matrix.npy', mtx)
# np.save('distortion_coefficients.npy', dist)

# # Mostra i risultati della calibrazione
# print("Matrice della camera:\n", mtx)
# print("Coefficienti di distorsione:\n", dist)

# cv2.destroyAllWindows()

import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
chessboard_size = (7, 9)  # Numero di angoli interni della scacchiera

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

images = glob.glob('image*.png')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

# # Imposta il formato di stampa per evitare la notazione scientifica
np.set_printoptions(suppress=True)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("Matrice della camera:\n", mtx)

cv.destroyAllWindows()
