string name # application name
string message # description of the status if required
# Possible status of startup process
byte RUNNING=0
byte WAITING=1
byte TIMEOUT=2
byte ABORTED=3 #Application not running
byte status # status of the startup process for the application. Enumerated above 

