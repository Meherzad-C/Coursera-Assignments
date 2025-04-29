# prerequisites
sudo apt update
sudo apt install -y gcc-7 g++-7

# building main
make clean && make
# run
./main

# building test
make clean && make test &&
# run
./test