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


#ifndef SFGE_SCENE_H
#define SFGE_SCENE_H


#include <list>

#include <engine/engine.h>
#include <utility/json_utility.h>

namespace sfge
{

class GameObject;
class Scene;

/**
* \brief The Scene Manager do the transition between two scenes, read from the Engine Configuration the scenes build list
*/
class SceneManager : public Module
{
public:
	using Module::Module;
	/**
	* \brief Initialize the SceneManager, get the Configuration from Engine and save the Scene lists from it
	*/
	void Init() override;
	/**
	* \brief Update the SceneManager, mostly updating the GameObjects of the current Scene and doing the transition when needed
	* \param dt Delta time since last frame
	*/
	void Update(sf::Time dt) override;
	/**
	* \brief Finalize and delete everything created in the SceneManager
	*/
	void Destroy() override;
	void LoadScene(std::string sceneName);
	/**
	* \brief Load a Scene and create all its GameObject
	* \param sceneName the scene path given by the configuration
	* \return the heap Scene that is automatically destroyed when not used
	*/
	std::shared_ptr<Scene> LoadSceneFromName(std::string sceneName);
	/**
	* \brief Load a Scene and create all its GameObject
	* \param sceneName the scene path given by the configuration
	* \return the heap Scene that is automatically destroyed when not used
	*/
	std::shared_ptr<Scene> LoadSceneFromJson(json& sceneJson);

	void SetCurrentScene(std::string sceneName);

	void SetCurrentScene(std::shared_ptr<Scene> scene);

	void Reset() override;
	void Collect() override;

	std::shared_ptr<Scene> GetCurrentScene();
private:
	std::shared_ptr<Scene> m_PreviousScene = nullptr;
	std::shared_ptr<Scene> m_CurrentScene = nullptr;
	//std::list<std::shared_ptr<Scene>> m_Scenes;

	bool m_Switching = false;
};

/**
* \brief The Scene includes GameObjects that are loaded from a JSON file
*
*/
class Scene
{
public:
	Scene(SceneManager* sceneManager);
	/**
	* \brief Update the Scene, mostly updating the GameObjects and doing the transition when needed
	* \param dt Delta time since last frame
	*/
	void Update(sf::Time dt);
	/**
	* \brief Destroy all the GameObjects of the scene
	*/
	~Scene();

	std::list<GameObject*>& GetGameObjects();
protected:
	std::string name;
	std::list<GameObject*> m_GameObjects;
	SceneManager* m_SceneManager;
	friend class SceneManager;
};
}
#endif
