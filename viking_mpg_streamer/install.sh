sudo cp /home/ubuntu/viking_mpg_streamer/dist/Release/GNU-Linux-x86/viking_mpg_streamer /usr/bin
sudo cp /home/ubuntu/viking_mpg_streamer/dist/Release/GNU-Linux-x86/input_uvc.so /usr/lib
sudo cp /home/ubuntu/viking_mpg_streamer/dist/Release/GNU-Linux-x86/output_http.so /usr/lib
sudo rm -f /usr/local/www/*.*
sudo chmod 755 /usr/local/www
sudo cp  /home/ubuntu/viking_mpg_streamer/dist/Release/GNU-Linux-x86/www/*.* /usr/local/www
sudo chmod 755 /usr/local/www/*.*

