extends ShapeCast3D
class_name DriveElement

# control variables
@export_flags_3d_physics var mask : int = 1
@export var castTo : Vector3 = Vector3(0,-1,0)
@export var springMaxForce : float = 300.0
@export var springForce : float = 180.0
@export var stifness : float = 0.85
@export var damping : float = 0.05
@export var Xtraction : float = 1.0
@export var Ztraction : float = 0.15
@export var staticSlideThreshold : float = 0.005
@export var massKG : float = 100.0

# public variables
var instantLinearVelocity : Vector3

# private variables
@onready var parentBody : RigidBody3D = get_parent()

var previousDistance : float = abs(castTo.y)
var previousHit : ShapeCastResult = ShapeCastResult.new()
var collisionPoint : Vector3 = castTo
var grounded : bool = false

var collision_index = 0

# shape cast result storage class
class ShapeCastResult:
	var is_hit : bool
	var hit_distance : float
	var hit_position : Vector3
	var hit_normal : Vector3
	var hit_point_velocity : Vector3
	var hit_body : PhysicsBody3D

func shape_cast():
	var result : ShapeCastResult = ShapeCastResult.new()
	if get_collision_count() == 0:
		result.is_hit = false
		return result
	elif get_collision_count() == 1:
		collision_index = 0
	elif get_collision_count() > 1:
		# Find the closest collision
		collision_index = 0
		var min_distance: float = -INF
		for i in range(get_collision_count()):
			var distance = (global_transform.origin + target_position - get_collision_point(i)).length()
			if distance < min_distance:
				min_distance = distance
				collision_index = i
	result.is_hit = true
	result.hit_distance = (global_transform.origin + target_position - get_collision_point(collision_index)).length()
	result.hit_position = get_collision_point(collision_index)
	
	DrawLine3D.DrawCube(get_collision_point(collision_index), 0.04, Color(255,255,0))
	
	result.hit_normal = get_collision_normal(collision_index)
	result.hit_point_velocity = Vector3.ZERO
	result.hit_body = null
	if get_collider_rid(collision_index):
		result.hit_body = instance_from_id(PhysicsServer3D.body_get_object_instance_id(get_collider_rid(collision_index)))
		var hitBodyState := PhysicsServer3D.body_get_direct_state(get_collider_rid(collision_index))
		var hitBodyPoint : Vector3 = get_collision_point(collision_index)
		result.hit_point_velocity = hitBodyState.get_velocity_at_local_position(hitBodyPoint * hitBodyState.transform)
		if GameState.debugMode:
			DrawLine3D.DrawRay(result.hit_position, result.hit_point_velocity, Color(0,0,0))
	return result
	
# set forward friction (braking)
func apply_brake(amount : float = 0.0) -> void:
	Ztraction = max(0.0, amount)

# function for applying drive force to parent body (if grounded)
func apply_force(force : Vector3) -> void:
	if is_colliding():
		parentBody.apply_force(force, get_collision_point(collision_index) - parentBody.global_transform.origin)

func _physics_process(delta) -> void:
	# perform sphere cast
	var castResult = shape_cast()
	collisionPoint = castResult.hit_position
	if GameState.debugMode:
		DrawLine3D.DrawCube(global_transform.origin, 0.1, Color(255,0,255))
		DrawLine3D.DrawCube(global_transform.origin + target_position, 0.1, Color(255,128,255))
	if castResult.is_hit:
		# if grounded, handle forces
		grounded = true
#		collisionPoint = castResult.hit_position
		if GameState.debugMode:
			DrawLine3D.DrawCube(castResult.hit_position, 0.04, Color(0,255,255)) # cyan cube
			DrawLine3D.DrawRay(castResult.hit_position, castResult.hit_normal,Color(255,255,255))
		
		# obtain instantaneaous linear velocity
		instantLinearVelocity = (collisionPoint - previousHit.hit_position) / delta
		
		# apply spring force with damping force
		var curDistance : float = castResult.hit_distance
		var FSpring : float = stifness * (abs(target_position.y) - curDistance) 
		var FDamp : float = damping * (previousDistance - curDistance) / delta
		var suspensionForce : float = clamp((FSpring + FDamp) * springForce,0,springMaxForce)
		var suspensionForceVec : Vector3 = castResult.hit_normal * suspensionForce
		
		# obtain axis velocity
		var localVelocity : Vector3 = (instantLinearVelocity - castResult.hit_point_velocity) * global_transform.basis 
		
		# axis deceleration forces based on this drive elements mass and current acceleration
		var XAccel : float = (-localVelocity.x * Xtraction) / delta
		var ZAccel : float = (-localVelocity.z * Ztraction) / delta
		var XForce : Vector3 = global_transform.basis.x * XAccel * massKG
		var ZForce : Vector3 = global_transform.basis.z * ZAccel * massKG
		
		# counter sliding by negating off axis suspension impulse at very low speed
		var vLimit : float = instantLinearVelocity.length_squared() * delta
		if vLimit < staticSlideThreshold:
#			suspensionForceVec = Vector3.UP * suspensionForce
			XForce.x -= suspensionForceVec.x * parentBody.global_transform.basis.y.dot(Vector3.UP)
			ZForce.z -= suspensionForceVec.z * parentBody.global_transform.basis.y.dot(Vector3.UP)
		
		# final impulse force vector to be applied
		var finalForce = suspensionForceVec + XForce + ZForce
		
		# draw debug lines
		if GameState.debugMode:
			DrawLine3D.DrawRay(get_collision_point(collision_index),suspensionForceVec/GameState.debugRayScaleFac,Color(0,255,0))
			DrawLine3D.DrawRay(get_collision_point(collision_index),XForce/GameState.debugRayScaleFac,Color(255,0,0))
			DrawLine3D.DrawRay(get_collision_point(collision_index),ZForce/GameState.debugRayScaleFac,Color(0,0,255))
			
		# apply forces relative to parent body
		var f:Vector3 = get_collision_point(collision_index) - parentBody.global_transform.origin
		parentBody.apply_force(finalForce, f)
		
		# apply forces to body affected by this drive element (action = reaction)
		if castResult.hit_body && castResult.hit_body is RigidBody3D:
			castResult.hit_body.apply_force(-finalForce, get_collision_point(collision_index) - castResult.hit_body.global_transform.origin)
		
		# set the previous values at the very end, after they have been used
		previousDistance = curDistance
		previousHit = castResult
	else:
		# not grounded, set prev values to fully extended suspension
		grounded = false
		previousHit = ShapeCastResult.new()
		previousHit.hit_position = global_transform.origin + target_position
		previousHit.hit_distance = abs(target_position.y)
		previousDistance = previousHit.hit_distance
		instantLinearVelocity = Vector3.ZERO

	#if Input.is_action_just_released("debug"):
		#print("%s
	#Is colliding: %s
	#Collision count:%s
	#Closest Safe Fraction: %s
	#Closest Unsafe Fraction: %s
	#Point: %s
	#Normal: %s
	#HIT DISTANCE: %s" % 
		#[
			#name, 
			#is_colliding(),
			#get_collision_count(),
			#get_closest_collision_safe_fraction(),
			#get_closest_collision_unsafe_fraction(),
			#get_collision_point(collision_index),
			#get_collision_normal(collision_index),
			#castResult.hit_distance
		#])
