# home_air_env

## env

```:console
$ pip install -r requirements.txt
$ cp env.py.sample env.py
```

* edit each server info
  * InfluxDB server
  * execution raspi environment

## execution

### stdout

```:console
$ python AirEnv.py
```

### post InfluxDB

```:console
$ python post_air_env.py
```

### ex. crontab

```:cron
* * * * * python /home/pi/home_air_env/post_air_env.py
```