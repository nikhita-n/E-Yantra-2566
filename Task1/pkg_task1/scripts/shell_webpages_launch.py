#!/usr/bin/env python

import requests

parameters = {"id":"task1", "team_id":"VB_2566", "unique_id":"rSnNRsNn", "turtle_x":123, "turtle_y":123, "turtle_theta":123, "value":36} 
URL2 = "https://script.google.com/macros/s/AKfycbzShBXxDAmHipGUVOBcP3OfhMabCBxfZD2dvvn0Abbw0Z84_PJu/exec"

response = requests.get(URL2, params=parameters)

print(response.content)
