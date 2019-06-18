/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <p2contact.h>
#include "p2body.h"
#include "p2matrix.h"
#include "../../Remotery/Remotery.h"

p2Collider* p2Contact::GetColliderA()
{
	return &m_ColliderA;
}

p2Collider* p2Contact::GetColliderB()
{
	return &m_ColliderB;
}

void p2Contact::SetColliderA(p2Collider collider)
{
	m_ColliderA = collider;
}

void p2Contact::SetColliderB(p2Collider collider)
{
	m_ColliderB = collider;
}

void p2ContactManager::Init(p2ContactListener* contactListener)
{
	m_ContactListener = contactListener;
}


void p2ContactManager::checkContact3(p2Body* bodies, std::vector<p2Body*> listBody)
{


	if (bodies->instanciate == true) //évite de tester des body pas instancié
	{
		for (int j = 0; j < listBody.size(); j++)
		{
			if (listBody[j]->instanciate == true)
			{
				if (bodies->GetPosition() == listBody[j]->GetPosition())
				{
					continue;
				}

				if (conditionContact(*bodies, *listBody[j]))
				{
					p2Mat22 mtv;	
					mtv.rows[0] = CheckCircleShape(*bodies, *listBody[j]);
					if (mtv.rows[0] == p2Vec2(0, 0))continue;
					if (mtv.rows[0] == p2Vec2(1, 1))
					{
						mtv.rows[0] = CheckRectShape(*bodies, *listBody[j]);

						if (mtv.rows[0] == p2Vec2(0, 0))continue;
						if (mtv.rows[0] == p2Vec2(1, 1))
						{
							mtv = CheckMTV(*bodies, *listBody[j]);
						}
					}
					
					p2Contact* contactptr = CheckContactList(bodies->GetCollider().at(0),
					                                         listBody[j]->GetCollider().at(0));

					if (contactptr != nullptr)continue;				
					bodies->SetLinearVelocity(bodies->GetLinearVelocity()-(mtv.rows[0].Normalized() *  p2Vec2::Dot(bodies->GetLinearVelocity(),
					mtv.rows[0].Normalized()) *2));
					listBody[j]->SetLinearVelocity(listBody[j]->GetLinearVelocity() - (mtv.rows[0].Normalized() *  p2Vec2::Dot(listBody[j]->GetLinearVelocity(),
					mtv.rows[0].Normalized()) * 2));
					if(bodies->GetType() != p2BodyType::STATIC)
					bodies->SetPosition(bodies->GetPosition() - mtv.rows[0]);
					if (listBody[j]->GetType() != p2BodyType::STATIC)
					listBody[j]->SetPosition(listBody[j]->GetPosition() + mtv.rows[0]);
					p2contact.SetColliderA(bodies->GetCollider().at(0));
					p2contact.SetColliderB(listBody[j]->GetCollider().at(0));

					m_ContactListener->BeginContact(&p2contact);

					contactList.push_back(p2contact);
					
				}
				else
				{
					p2Contact* contactptr = CheckContactList(bodies->GetCollider().at(0),
					                                         listBody[j]->GetCollider().at(0));

					if (contactptr->GetColliderA() == nullptr)continue;
					if (contactptr->GetColliderA()->GetShape() == nullptr)continue;

					m_ContactListener->EndContact(contactptr);

					*contactptr = p2Contact();
				}
			}
		}
	}
}

p2Contact* p2ContactManager::CheckContactList(p2Collider colliderA, p2Collider colliderB)
{
	for (auto& p2_contact : contactList)
	{
		if (p2_contact.GetColliderA()->GetUserData() == colliderA.GetUserData() &&
			p2_contact.GetColliderB()->GetUserData() == colliderB.GetUserData() ||
			p2_contact.GetColliderB()->GetUserData() == colliderA.GetUserData() &&
			p2_contact.GetColliderA()->GetUserData() == colliderB.GetUserData())
		{
			return &p2_contact;
		}
	}
	return nullptr;
}

bool p2ContactManager::conditionContact(p2Body body, p2Body body2)
{
	if (((body.GetAABB().topRight > body2.GetAABB().bottomLeft) &&
			(body.GetAABB().topRight < body2.GetAABB().topRight)) ||
		((body.GetAABB().bottomLeft > body2.GetAABB().bottomLeft) &&
			(body.GetAABB().bottomLeft < body2.GetAABB().topRight)) ||
		((body.GetAABB().topRight.x > body2.GetAABB().bottomLeft.x &&
				body.GetAABB().bottomLeft.y > body2.GetAABB().bottomLeft.y) &&
			(body.GetAABB().topRight.x < body2.GetAABB().topRight.x &&
				body.GetAABB().bottomLeft.y < body2.GetAABB().topRight.y)) ||
		((body.GetAABB().bottomLeft.x > body2.GetAABB().bottomLeft.x &&
				body.GetAABB().topRight.y > body2.GetAABB().bottomLeft.y) &&
			(body.GetAABB().bottomLeft.x < body2.GetAABB().topRight.x &&
				body.GetAABB().topRight.y < body2.GetAABB().topRight.y))) //test si les objets se touchent
	{
		return true;
	}
	if (((body2.GetAABB().topRight > body.GetAABB().bottomLeft) &&
			(body2.GetAABB().topRight < body.GetAABB().topRight)) ||
		((body2.GetAABB().bottomLeft > body.GetAABB().bottomLeft) &&
			(body2.GetAABB().bottomLeft < body.GetAABB().topRight)) ||
		((body2.GetAABB().topRight.x > body.GetAABB().bottomLeft.x &&
				body2.GetAABB().bottomLeft.y > body.GetAABB().bottomLeft.y) &&
			(body2.GetAABB().topRight.x < body.GetAABB().topRight.x &&
				body2.GetAABB().bottomLeft.y < body.GetAABB().topRight.y)) ||
		((body2.GetAABB().bottomLeft.x > body.GetAABB().bottomLeft.x &&
				body2.GetAABB().topRight.y > body.GetAABB().bottomLeft.y) &&
			(body2.GetAABB().bottomLeft.x < body.GetAABB().topRight.x &&
				body2.GetAABB().topRight.y < body.GetAABB().topRight.y))) //test si les objets se touchent
	{
		return true;
	}

	return false;
}

p2Mat22 p2ContactManager::CheckMTV(p2Body body, p2Body body2)
{
	p2Mat22 mtvx;
	p2Mat22 mtvy;
	p2Vec2 distanceAB = body.GetPosition() - body2.GetPosition();

	p2Vec2 sizeSomme = body.GetAABB().GetExtends() / 2 + body2.GetAABB().GetExtends() / 2;


	if (abs(distanceAB.x - sizeSomme.x) < abs(distanceAB.x + sizeSomme.x))
	{
		mtvx = p2Mat22(p2Vec2((distanceAB.x - sizeSomme.x), 0),
		               body.GetPosition() + p2Vec2(body.GetAABB().GetExtends().x / 2, 0));
	}
	else
	{
		mtvx = p2Mat22(p2Vec2((distanceAB.x + sizeSomme.x), 0),
		               body.GetPosition() + p2Vec2(body.GetAABB().GetExtends().x / 2, 0));
	}

	if (abs(distanceAB.y - sizeSomme.y) < abs(distanceAB.y + sizeSomme.y))
	{
		mtvy = p2Mat22(p2Vec2(0, (distanceAB.y - sizeSomme.y)),
		               body.GetPosition() + p2Vec2(0, body.GetAABB().GetExtends().y / 2));
	}
	else
	{
		mtvy = p2Mat22(p2Vec2(0, (distanceAB.y + sizeSomme.y)),
		               body.GetPosition() + p2Vec2(0, body.GetAABB().GetExtends().y / 2));
	}

	if (mtvx.rows[0].GetMagnitude() > mtvy.rows[0].GetMagnitude() && mtvy != p2Mat22())
	{
		return mtvy;
	}
	else
	{
		return mtvx;
	}

	return p2Mat22();
}

p2Vec2 p2ContactManager::CheckCircleShape(p2Body bodyA, p2Body bodyB)
{
	if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(bodyA.GetCollider().at(0).GetShape()))
	{
		if (p2CircleShape* circleshape2 = dynamic_cast<p2CircleShape*>(bodyB.GetCollider().at(0).GetShape()))
		{
			if( (bodyA.GetPosition() - bodyB.GetPosition()).GetMagnitude()< circleshape->GetRadius() +circleshape2->GetRadius())
			{
				return (bodyA.GetPosition()-bodyB.GetPosition()).Normalized()*
					(circleshape->GetRadius()+circleshape2->GetRadius()-(bodyA.GetPosition()-bodyB.GetPosition()).GetMagnitude());
			}
			else
			{
				return p2Vec2(0,0);
			}
		}
	}

	
	return p2Vec2(1,1);
}

p2Vec2 p2ContactManager::CheckRectShape(p2Body bodyA, p2Body bodyB)
	{
	if (p2RectShape* rectShape = dynamic_cast<p2RectShape*>(bodyA.GetCollider().at(0).GetShape()))
	{
		if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(bodyB.GetCollider().at(0).GetShape()))
		{
			std::vector<p2Vec2> corner;
			corner.push_back(bodyA.GetAABB().bottomLeft);
			corner.push_back(bodyA.GetAABB().topRight);
			corner.push_back(p2Vec2(bodyA.GetAABB().bottomLeft.x, bodyA.GetAABB().topRight.y));
			corner.push_back(p2Vec2(bodyA.GetAABB().topRight.x, bodyA.GetAABB().bottomLeft.y));
			p2Vec2 axe;
			if(bodyB.GetPosition().x>bodyA.GetAABB().bottomLeft.x&&bodyB.GetPosition().x<bodyA.GetAABB().topRight.x)
			{
				axe = p2Vec2(0,rectShape->GetSize().y);
			}

			else  if (bodyB.GetPosition().y > bodyA.GetAABB().bottomLeft.y&&bodyB.GetPosition().y < bodyA.GetAABB().topRight.y)
			{
				axe = p2Vec2(rectShape->GetSize().x,0);
			}
			else
			{
				
				float minDist = 1000;
				for (auto corners : corner)
				{
					if(minDist>(bodyB.GetPosition()-corners).GetMagnitude())
					{
						minDist = (bodyB.GetPosition() - corners).GetMagnitude();
						axe = (bodyA.GetPosition() - corners);

					}
				}
			}
			axe = axe.Normalized()*circleshape->GetRadius();
			p2Vec2 mtve = rectShape->GetSize();
			for (auto corn : corner)
			{
				p2Vec2 cornerPause =  corn - bodyB.GetPosition();
				float projection = p2Vec2::Dot(cornerPause, axe)/p2Vec2::Dot(axe,axe);
				if(projection<=1 &&projection>=-1)
				{
					if (projection>0&&mtve.GetMagnitude()>(axe*-(1-projection)).GetMagnitude())
					{
						mtve = axe * -(1-projection);
					}
					else if (mtve.GetMagnitude() > (axe*(1 + projection)).GetMagnitude())
					{
						
						mtve = axe * (1+projection);
					}

				}
			}
			if (mtve == rectShape->GetSize())
			{
				return p2Vec2(0, 0);
			}
			return mtve;
		}
	}

	if (p2RectShape* rectShape = dynamic_cast<p2RectShape*>(bodyB.GetCollider().at(0).GetShape()))
	{
		if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(bodyA.GetCollider().at(0).GetShape()))
		{
			std::vector<p2Vec2> corner;
			corner.push_back(bodyB.GetAABB().bottomLeft);
			corner.push_back(bodyB.GetAABB().topRight);
			corner.push_back(p2Vec2(bodyB.GetAABB().bottomLeft.x, bodyB.GetAABB().topRight.y));
			corner.push_back(p2Vec2(bodyB.GetAABB().topRight.x, bodyB.GetAABB().bottomLeft.y));
			p2Vec2 axe;
			if (bodyA.GetPosition().x > bodyB.GetAABB().bottomLeft.x&&bodyA.GetPosition().x < bodyB.GetAABB().topRight.x)
			{
				axe = p2Vec2(0,rectShape->GetSize().y);
			}

			else if (bodyB.GetPosition().y > bodyA.GetAABB().bottomLeft.y&&bodyA.GetPosition().y < bodyB.GetAABB().topRight.y)
				{
					axe = p2Vec2( rectShape->GetSize().x,0);
				}
				else
				{

					float minDist = 1000;
					for (auto corners : corner)
					{
						if (minDist > (bodyA.GetPosition() - corners).GetMagnitude())
						{
							minDist = (bodyA.GetPosition() - corners).GetMagnitude();
							axe = (bodyB.GetPosition() - corners);

						}
					}
				}
			axe = axe.Normalized()*circleshape->GetRadius();
			p2Vec2 mtve = rectShape->GetSize();
			for (auto corn : corner)
			{
				p2Vec2 cornerPause =   corn - bodyA.GetPosition();
				float projection = p2Vec2::Dot(cornerPause, axe) / p2Vec2::Dot(axe, axe);
				if (projection <= 1 && projection >= -1)
				{
					if (projection > 0 && mtve.GetMagnitude() > (axe*-(1 - projection)).GetMagnitude())
					{
						mtve = axe * -(1 - projection);
					}
					else if (mtve.GetMagnitude() > (axe*(1 + projection)).GetMagnitude())
					{

						mtve = axe * (1 + projection);
					}

				}
			}
			if (mtve == rectShape->GetSize())
			{
				return p2Vec2(0, 0);
			}
			return mtve;
		}
	}

	return p2Vec2(1, 1);
}

