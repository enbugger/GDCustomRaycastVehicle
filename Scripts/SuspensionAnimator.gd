extends MeshInstance3D

# public variables
@export var wheelOffset : Vector3 = Vector3(0,0.7,0)
@export var trackThickness : float = 0.05
@export var returnSpeed : float = 6.0
@export var boneName : String
@export var raycastPath: NodePath
@export var trackSkeletonPath: NodePath

# private variables
var raycast:ShapeCast3D
var trackSkeleton : Skeleton3D
var trackBone
var trackOffset : Vector3 = Vector3(0,trackThickness,0)

func _ready() -> void:
	# setup references
	raycast = get_node(raycastPath)
	trackSkeleton = get_node(trackSkeletonPath)
	trackBone = trackSkeleton.find_bone(boneName)
	if boneName == null:
		boneName = self.name
	
func _physics_process(delta) -> void:
	# set the wheel position
	if raycast.is_colliding():
		transform.origin.y = (raycast.to_local(raycast.get_collision_point(0)) + wheelOffset).y
	#else:
		#transform.origin.y = lerp(transform.origin.y, (raycast.target_position + wheelOffset).y, returnSpeed * delta)
	# deform the track based on wheel position
	var tbonePos = trackSkeleton.get_bone_global_pose(trackBone)
	tbonePos.origin = (global_transform.origin + trackOffset) * trackSkeleton.global_transform
	trackSkeleton.set_bone_global_pose_override(trackBone, tbonePos, 1.0, true)
	
