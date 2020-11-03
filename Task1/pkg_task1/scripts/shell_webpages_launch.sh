#!/bin/bash

# Store URL in a variable
URL1="http://www.hivemq.com/demos/websocket-client/"
URL2="https://docs.google.com/spreadsheets/d/1R700enVYAQB4rxRhFcQFxA6m84VA73jnXxvUcPj12XU/edit?usp=sharing"

# Print some message
echo "** Opening $URL1 and $URL2 in Firefox **"

# Use firefox to open the URLs in a new window
firefox $URL1 $URL2