# donkeyDrift
This is a new project which is based on donkeyCar.
We have already modified the video capture algorithm and locked tensorflow's version at tensorflow-1.8.0 and cloned the donkeycar version with 2.5.1 branch.
* You can download the model that we have already trained at local.
* You can join us and give us some advices. 
## How to use it
* clone the repository by:
* git clone git@github.com:Mavengers/DonkeyDrift.git
* cd donkeyDrift/
* chmod +x setup.sh
* ./setup.sh
## How to create a new car instance. 
*  $donkey createcar  mynewcar
## How to train my car.
* $python manager_Karasliner.py train --tub=./data/ --model=./models/mytrainning.model
## How to dirve my car.
* $python manager_Karasliner.py drive 
## Have fun!

