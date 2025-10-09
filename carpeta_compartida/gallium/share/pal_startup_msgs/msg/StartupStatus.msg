string name # application name
string message # description of the status if required
# Possible status of startup process
string MISSING=Missing
string RUNNING=Running
string PENDING=Pending
string CRASHED=Crashed
string STOPPED=Stopped
string ABORTED=Aborted
string TIMEOUT=Timeout
string MANUAL=Manual
string FINISHED=Finished
string status # status of the startup process for the application. Enumerated above 
string startup_id # Id of the node starting the app (ie: control, multimedia)

