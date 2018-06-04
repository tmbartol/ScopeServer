#!/usr/bin/env python3
import os
import sys


# Get IP address
import netifaces as ni
def get_ip(iface = 'wlan0'):
  try:
    ni.ifaddresses(iface)
    ip = ni.ifaddresses(iface)[ni.AF_INET][0]['addr']
  except:
    ip = '127.0.0.1'
  return ip


if __name__ == "__main__":
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "scopeserver.settings")
    try:
        from django.core.management import execute_from_command_line
    except ImportError:
        # The above import may fail for some other reason. Ensure that the
        # issue is really that Django is missing to avoid masking other
        # exceptions on Python 2.
        try:
            import django
        except ImportError:
            raise ImportError(
                "Couldn't import Django. Are you sure it's installed and "
                "available on your PYTHONPATH environment variable? Did you "
                "forget to activate a virtual environment?"
            )
        raise

#    execute_from_command_line(sys.argv)

    server_args  = [sys.argv[0], "runserver", get_ip() + ":8000"]
    execute_from_command_line(server_args)


