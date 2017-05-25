import bluetooth


#nearby_devices = bluetooth.discover_devices()
#print nearby_devices
srv = bluetooth.find_service(name='GamePad', address='00:90:E1:9B:6D:C7')
print srv
