#!/bin/bash

systemctl stop scopeserver_control.service
systemctl stop scopeserver_webapp.service
systemctl stop timeservice.service

systemctl start timeservice.service
systemctl start scopeserver_webapp.service
systemctl start scopeserver_control.service
