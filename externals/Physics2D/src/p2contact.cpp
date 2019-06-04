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
p2Collider * p2Contact::GetColliderA()
{
	return &m_ColliderA;
}

p2Collider * p2Contact::GetColliderB()
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

void p2ContactManager::Init(p2ContactListener * contactListener)
{
	m_ContactListener = contactListener;
}
void p2ContactManager::checkContact(std::vector<p2Body*> bodies)
{
	for (int i = 0 ; i<bodies.size();i++)
	{
		if (bodies[i]->instanciate == true) //évite de tester des body pas instancié
		{
			for (int j = i ; j<bodies.size();j++)
			{
				if (bodies[j]->instanciate == true)
				{
					if (bodies[i]->GetPosition() == bodies[j]->GetPosition())
					{
						continue;
					}
					if (conditionContact(*bodies[i], *bodies[j]))
					{
						//if (CheckCircleShape(bodies[i], bodies[j])==true)
						//{
							p2Contact* contactptr = CheckContactList(bodies[i]->GetCollider().at(0), bodies[j]->GetCollider().at(0));

							if (contactptr != nullptr)continue;

							p2contact.SetColliderA(bodies[i]->GetCollider().at(0));
							p2contact.SetColliderB(bodies[j]->GetCollider().at(0));

							m_ContactListener->BeginContact(&p2contact);

							contactList.push_back(p2contact);
						//}
					
					}
					else
					{
						p2Contact* contactptr = CheckContactList(bodies[i]->GetCollider().at(0), bodies[j]->GetCollider().at(0));

						if (contactptr == nullptr)continue;

						m_ContactListener->EndContact(contactptr);

						*contactptr = p2Contact();
					}
				}
			}
		}
	}
}
void p2ContactManager::checkContact2(p2Body* bodiesA, p2Body* bodiesB)
{

	if (bodiesA->instanciate == true) //évite de tester des body pas instancié
	{

		if (bodiesB->instanciate == true)
		{
			if (bodiesA->GetPosition() == bodiesB->GetPosition())
			{
				return;
			}
			if (conditionContact(*bodiesA, *bodiesB))
			{
				//if (CheckCircleShape(bodies[i], bodies[j])==true)
				//{
				p2Contact* contactptr = CheckContactList(bodiesA->GetCollider().at(0), bodiesB->GetCollider().at(0));

				if (contactptr != nullptr)return;

				p2contact.SetColliderA(bodiesA->GetCollider().at(0));
				p2contact.SetColliderB(bodiesB->GetCollider().at(0));

				m_ContactListener->BeginContact(&p2contact);

				contactList.push_back(p2contact);
				//}

			}
			else
			{
				p2Contact* contactptr = CheckContactList(bodiesA->GetCollider().at(0), bodiesB->GetCollider().at(0));

				if (contactptr == nullptr)return;

				m_ContactListener->EndContact(contactptr);

				*contactptr = p2Contact();
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

/*bool p2ContactManager::CheckCircleShape(p2Body bodyA, p2Body bodyB)
{
	if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(bodyA.GetCollider().at(0).GetShape()))
	{
		if (p2CircleShape* circleshape2 = dynamic_cast<p2CircleShape*>(bodyB.GetCollider().at(0).GetShape()))
		{
			if( (bodyA.GetPosition() - bodyB.GetPosition()).GetMagnitude() > circleshape->GetRadius() +circleshape2->GetRadius())
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
	if (p2RectShape* rectShape = dynamic_cast<p2RectShape*>(bodyA.GetCollider().at(0).GetShape()))
	{
		if (p2RectShape* rectShape2 = dynamic_cast<p2RectShape*>(bodyB.GetCollider().at(0).GetShape()))
		{
			if (bodyA.GetPosition().x < bodyB.GetPosition().x + rectShape->GetSize().x &&
				bodyA.GetPosition().x + rectShape->GetSize().x > bodyB.GetPosition().x &&
				bodyA.GetPosition().y < bodyB.GetPosition().y + rectShape->GetSize().y &&
				bodyA.GetPosition().y + rectShape->GetSize().y > bodyB.GetPosition().y)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
	return true;
}*/

