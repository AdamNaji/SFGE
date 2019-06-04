#include "..\include\p2collider.h"


void p2Collider::Init(p2ColliderDef* colliderDef)
{
	userData = colliderDef->userData;
	shape = colliderDef->shape;
	isSensor = colliderDef->isSensor;
	restitution = colliderDef->restitution;
	
}
bool p2Collider::IsSensor() const
{
	return isSensor;
}

void * p2Collider::GetUserData() const
{
	return userData;
}

p2Shape* p2Collider::GetShape() const
{
	return shape;
}

void p2Collider::SetUserData(void* colliderData)
{
	userData = colliderData;
}

p2AABB p2Collider::GetAABB(p2Vec2 position)
{
	p2AABB aabb;
	if (shape == nullptr)
	{
		return aabb;
	}
	if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(shape))
	{
		aabb.SetCenterAndExtends(position, p2Vec2(circleshape->GetRadius(), circleshape->GetRadius()));
		
	}
	else if (p2RectShape* rectshape = dynamic_cast<p2RectShape*>(shape))
	{
		aabb.SetCenterAndExtends(position, rectshape->GetSize());
	}
	return aabb;
}