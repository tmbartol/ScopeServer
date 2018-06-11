#!/bin/bash

systemctl stop scopeserver_webapp.service
systemctl stop scopeserver_control.service
systemctl stop timeservice.service

systemctl start timeservice.service
sleep 5
systemctl start scopeserver_control.service
systemctl start scopeserver_webapp.service
