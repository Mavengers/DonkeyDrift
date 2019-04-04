# DonkeyDrift
This is a new project from Shanghai, China, which is originally based on DonkeyCar.
We have built an Arduino chassis with high performance, modified several configurations and chosen tensorflow version at 1.8.0 and donkeycar version at 2.5.1 of master branch.

# Instruction

## 1. Reference & Tools:

| https://pan.baidu.com/s/1ZSSp5YRDbOhoB2DSR1p4Hw |
| ----------------------------------------------- |
| Password：3pfj                                  |

## 2. Clone this repository:

```
git clone https://github.com/Haobot/DonkeyDrift

cd DonkeyDrift/
```

## 3. Install DonkeyDrift:

```
activate donkey
pip install -e .
```

## 4. Create new instance:

```
donkey createcar ./mycar
cd  mycar/
```

## 5. Calibrate car:

```
donkey calibrate --channel 0
donkey calibrate --channel 1
```

## 6. Drive car:

```
python manage.py drive 
```

## 7. Train Model on PC or Server:
```
python manage.py train --tub=./data/your_tub_name --model=./models/your_model_name
```

## 8. Test Model with Unity simulators:

```
donkey sim --model=./models/your_model_name --type=linear
```

**P.S. Simulators just work well on Linux and MAC now, Windows version need fixing.**

## 9. Drive car with model for Auto Drive:

```
python manage.py drive --model=./models/your_model_name
```

------

# Have fun!
