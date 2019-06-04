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

#include <extensions/aabb_test.h>
#include <engine/engine.h>
#include <engine/config.h>
#include <graphics/graphics2d.h>
#include <physics/body2d.h>
#include <physics/physics2d.h>



namespace sfge::ext
{

	AabbTest::AabbTest(Engine& engine) :
		System(engine)
	{

	}
	
	void AabbTest::OnEngineInit()
	{
		m_Transform2DManager = m_Engine.GetTransform2dManager();
		m_Body2DManager = m_Engine.GetPhysicsManager()->GetBodyManager();
		m_Graphics2DManager = m_Engine.GetGraphics2dManager();
		m_PhysicsManager = m_Engine.GetPhysicsManager();
		m_World = m_PhysicsManager->GetWorld();
		quadTree = m_World->m_QuadTree;
		

		auto config = m_Engine.GetConfig();
		screenSize = sf::Vector2f(config->screenResolution.x, config->screenResolution.y);
		auto* entityManager = m_Engine.GetEntityManager();

		entities = entityManager->GetEntitiesWithType(ComponentType::BODY2D);
		for (auto i = 0u; i < entities.size(); i++)
		{
			auto body = m_Body2DManager->GetComponentPtr(entities[i]);
			bodies.push_back(body->GetBody());
		}
		p2AABB bounds;
		bounds.bottomLeft = p2Vec2(0, 0);
		bounds.topRight = pixel2meter(screenSize);
		quadTree->m_Bounds = bounds;
	}

	void AabbTest::OnUpdate(float dt)
	{
		(void)dt;
	}


	void AabbTest::OnFixedUpdate()
	{
		rmt_ScopedCPUSample(AabbTestFixedUpdate, 0);
		
	}

	void AabbTest::OnDraw()
	{
		rmt_ScopedCPUSample(AabbTestDraw, 0);
		
		for (auto& body : bodies)
		{
			DrawAABB(body->GetAABB());
		}
		DrawQuadTree(m_World->m_QuadTree);
		
		
	}

	void AabbTest::DrawAABB(p2AABB aabb)
	{
		const auto bottomRight = p2Vec2(aabb.topRight.x, aabb.topRight.y - aabb.GetExtends().y);
		const auto topLeft = p2Vec2(aabb.bottomLeft.x, aabb.bottomLeft.y + aabb.GetExtends().y);
		m_Graphics2DManager->DrawLine(meter2pixel(aabb.topRight), meter2pixel(bottomRight));
		m_Graphics2DManager->DrawLine(meter2pixel(aabb.topRight), meter2pixel(topLeft));
		m_Graphics2DManager->DrawLine(meter2pixel(aabb.bottomLeft), meter2pixel(bottomRight));
		m_Graphics2DManager->DrawLine(meter2pixel(aabb.bottomLeft), meter2pixel(topLeft));
	}
	void AabbTest::DrawQuadTree(p2QuadTree *quad_tree)
	{
		const auto aabb = quad_tree->GetAABB();
		DrawAABB(aabb);
		if (!quad_tree->GetChildren().empty())
		{
			for (auto& child : quad_tree->GetChildren())
			{
				DrawQuadTree(child);
			}
			
		}
	}

}
