#!/usr/bin/env python
import rospy
import seneka_pnp.srv


print "init",
rospy.init_node('seneka_demo')

rospy.wait_for_service('/seneka_pnp/getState')
rospy.wait_for_service('/seneka_pnp/setTransition')
print "..."

srv_getState = rospy.ServiceProxy('/seneka_pnp/getState', seneka_pnp.srv.getState)
srv_setTransisition = rospy.ServiceProxy('/seneka_pnp/setTransition', seneka_pnp.srv.setTransition)
print "done"


def getState():
	try:
	  resp = srv_getState()
	  return resp.state
	except rospy.ServiceException as exc:
	  print("Service did not process request: " + str(exc))
	return "error"
	
def setTransisition(tr):
	print "-> "+tr
	try:
	  resp = srv_setTransisition(tr)
	  return resp.transition==tr
	except rospy.ServiceException as exc:
	  print("Service did not process request: " + str(exc))
	return False

def setTransisitionAndWait(tr, exp, cur):
	setTransisition(tr)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		st = getState()
		if st==cur:
			rate.sleep()
		elif st==exp:
			return True
		else:
			print "could not reach expected state ("+exp+") != "+st
			return False
	return False
			
st=""
while not rospy.is_shutdown():
	st = getState()
	
	#at startup
	if st=="collision_free":
		setTransisitionAndWait("toHome", "home", st)
	elif st=="home":
		setTransisitionAndWait("homeToPreGraspRear", "pregrasp-rear", st)
		print "insert sensor node"
		input("Press Enter to continue...")
		
	#loop
	elif st=="pregrasp-rear" or st=="deployed-rear":
		setTransisitionAndWait("toPrePackRear", "prepack-rear", st)
	elif st=="prepack-rear":
		setTransisitionAndWait("toPackedRear", "packed-rear", st)
	elif st=="packed-rear":
		setTransisitionAndWait("deployRear", "deploy-rear", st)
	elif st=="deploy-rear":
		setTransisitionAndWait("deployRearDrop", "deployed-rear", st)
		
	#failure case
	else:
		print "unknown state: "+st
		print "please go back to a safe starting position"
		print "exiting..."
		exit()
print "last state was: ", st
