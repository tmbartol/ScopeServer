from django.shortcuts import render
from django.http import JsonResponse
from pdb import set_trace as debug

# Create your views here.

import time

def control(request):

	if request.is_ajax():
		result = {}
		action = request.POST.get("action", "")
		try:
			if action == "serverStatus":
				result['#gps_location_lat'] = str(time.time())
				result['#gps_location_lon'] = str(time.time())
				result['#local_time'] = str(time.time())
				result['#utc_time'] = str(time.time())
				result['#scope_position_dec'] = str(time.time())
				result['#scope_position_ra'] = str(time.time())
				result['status'] = "Success"
		except Exception as e:
			result['status'] = "Failure"
			result['msg'] = str(e)
		finally:
			return(JsonResponse(result))

	return(render(request, "server/control.html", locals()))