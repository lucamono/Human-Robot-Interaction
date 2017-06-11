Install libFreenect from https://github.com/OpenKinect/libfreenect

IMPORTANT: Version 0.4.3 is needed!!!

Follow instructions on README file, summarized below:
    
      sudo apt-get install git cmake build-essential libusb-1.0-0-dev
      sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
      git clone https://github.com/OpenKinect/libfreenect
      cd libfreenect
      git checkout v0.4.3
      mkdir build
      cd build
      cmake -L ..
      make
      sudo make install

To test if it works:

      bin/freenect-cppview 
      

