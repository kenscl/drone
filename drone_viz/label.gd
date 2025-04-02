extends Label


@onready var object = $BLock/Block
@onready var label := $CanvasLayer/Label
func _ready() -> void:
	pass



func _process(delta: float) -> void:
	var quat = object. basis.get_rotation_quaternion()
	text = "Quaternion: (%.3f, %.3f, %.3f, %.3f)" % [quat.w, quat.x, quat.y, quat.z]
