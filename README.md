To execute the project:

1. Download Docker
2. Download DroidCam on your phone
3. Put the WiFi IP showed in the homepage of DroidCam into the ENV VARIABLE `DROIDCAM_URL` into the docker-compose.yml -> gesture service
4. Download CoppeliaSim Edu, rename it in `CoppeliaSim.tar.xz` and put it into the coppeliasim folder.
5. `docker-compose up --build`

Then
* To see the simulation on CoppeliaSIM, visit `localhost:6080/vnc.html`
* To see the images of the gesture and apriltag detection, visit `localhost:3000`
