from django.shortcuts import render
from django.http import JsonResponse
from pdb import set_trace as debug

# Create your views here.

import datetime
import time

def control(request):

	if request.is_ajax():
		result = {}
		action = request.POST.get("action", "")
		try:
			if action == "serverStatus":
				result['#gps_location_lat'] = str(int(time.time()))
				result['#gps_location_lon'] = str(int(time.time()))
				result['#local_time'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-5]
				result['#utc_time'] = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-5]
				result['#scope_position_dec'] = str(int(time.time()))
				result['#scope_position_ra'] = str(int(time.time()))
				result['status'] = "Success"
		except Exception as e:
			result['status'] = "Failure"
			result['msg'] = str(e)
		finally:
			return(JsonResponse(result))

	return(render(request, "server/control.html", locals()))
