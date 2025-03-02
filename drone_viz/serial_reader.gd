extends Node3D

var websocket = WebSocketPeer.new()

func _ready():
	var err = websocket.connect_to_url("ws://localhost:1304")
	if err != OK:
		print("Unable to connect")
		set_process(false)
	else:
		await get_tree().create_timer(2).timeout

func _process(_delta):
	websocket.poll()
	var state = websocket.get_ready_state()


	if state == WebSocketPeer.STATE_OPEN:
		while websocket.get_available_packet_count():
			var data = websocket.get_packet().get_string_from_utf8()
			print(data)
			var quat = JSON.parse_string(data)
			
			if quat and quat.has("qw"):
				var qw = quat["qw"]
				var qi = quat["qi"]
				var qj = quat["qj"]
				var qk = quat["qk"]
				
				var q = Quaternion(qi, qj, qk, qw)
				print(q)
				rotation = q.get_euler()
