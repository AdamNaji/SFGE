#include "..\include\p2quadtree.h"
#include "../../Remotery/Remotery.h"

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds)
{
	// Set base values
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
}

p2QuadTree::~p2QuadTree()
{
}

void p2QuadTree::Clear()
{
	for (auto child : m_Children)
	{
		child->Clear();
	}
	m_Objects.clear();
	m_Children.clear();
}

void p2QuadTree::Split()
{
	if (m_NodeLevel > MAX_LEVELS)
		return;

	// Set the current position
	p2Vec2 currentPosition = m_Bounds.bottomLeft;

	// Define the size of the child sides depending on the amount of child tree number
	const auto childSize = (m_Bounds.topRight - currentPosition) / 2;

	for (auto x = 0u; x < 2; x++)
	{
		for (auto y = 0u; y < 2; y++)
		{
			p2AABB childAABB;

			currentPosition = m_Bounds.bottomLeft + p2Vec2(childSize.x * x, childSize.y * y);

			childAABB.bottomLeft = currentPosition;

			childAABB.topRight = {currentPosition.x + childSize.x, currentPosition.y + childSize.y};

			m_Children.push_back(new p2QuadTree(m_NodeLevel + 1, childAABB));
		}
	}
	for (int i = 0; i<m_Objects.size();i++)
	{
		for (auto* child : m_Children)
		{
			if (child->m_Bounds.DoOverlapWith(m_Objects[i]->GetAABB())) 
			{
				child->Insert(m_Objects[i]);
				m_Objects.erase(m_Objects.begin()+i);
				
				
			};
			if (m_Objects.size()<=i)
			{
				break;
			}
		}
	}

}

int p2QuadTree::GetIndex(p2Body* rect)
{
	for (int i = 0; i < MAX_CHILD_TREE_NMB; i++)
	{
		for (p2Body* body : m_Children[i]->m_Objects)
		{
			if (body == rect)
				return i;
		}
	}

	return 0;
}

void p2QuadTree::Insert(p2Body* obj)
{
	if (!m_Children.empty())
	{
		bool inserted = false;
		for (auto& child : m_Children)
		{
			if (child->m_Bounds.DoOverlapWith(obj->GetAABB()))
			{
				inserted = true;
				child->Insert(obj);
			}
		}
		if ( inserted== false)
		{
			m_Objects.push_back(obj);
		}
	}
	else
	{
		m_Objects.push_back(obj);
	}
	if (m_Objects.size()>MAX_OBJECTS)
	{
		if (m_Children.empty())
		{
			Split();
		}
	}
}

std::vector<p2Body*> p2QuadTree::Retrieve(p2Body* rect)
{
	rmt_ScopedCPUSample(retreve, 0);

	std::vector<p2Body*> returnValue;
	for (auto& body : m_Objects)
	{
		if (body==rect)
		{
			std::vector<p2Body*> childObject = GetChildrenObj();
			if (!childObject.empty()) {
				returnValue.insert(returnValue.begin(), childObject.begin(), childObject.end());
		
			}
		
		}
		
	}
	for (auto& child : m_Children)
	{
		const auto retrieve = child->Retrieve(rect);
		if (returnValue.empty())
		{
			returnValue = retrieve;
			
		}
		
	}
	return returnValue;
}
p2AABB p2QuadTree::GetAABB() const
{
	return m_Bounds;
}
std::vector<p2Body*> p2QuadTree::GetChildrenObj()
{
	std::vector<p2Body*> returnValue = m_Objects;
	for (auto chid : m_Children)
	{
		std::vector<p2Body*> childObject = chid->GetChildrenObj();
		if (!childObject.empty()) {
			returnValue.insert(returnValue.begin(), childObject.begin(), childObject.end());
		}
	}
	return returnValue;
}
std::vector<p2QuadTree*> p2QuadTree::GetChildren() const
{
	return m_Children;
}
