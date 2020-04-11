# coding=utf-8
from influxdb import InfluxDBClient
from AirEnv import AirEnv

from env import *

client = InfluxDBClient(INFLUX_URL, INFLUX_PORT, INFLUX_USER, INFLUX_PASS, DB_NAME)

# database exist
dbs = client.get_list_database()
sample_db = {u'name': DB_NAME}
if sample_db not in dbs:
    client.create_database(DB_NAME)

air = AirEnv()

air_param = air.readData()
co2 = air.mh_z19()
air_param.update(co2)

import_array = [{
    "fields": air_param,
    "tags": {
        "category": "air_environment",
        "host": HOSTNAME
    },
    "measurement": "metrics"
}]

client.write_points(import_array)
