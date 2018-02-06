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

#ifndef SFGE_ENGINE_H
#define SFGE_ENGINE_H

#include <SFML/Graphics/RenderWindow.hpp>

#include <memory>
#include <array>

#include <engine/config.h>
#include <utility/singleton.h>



namespace sfge
{

/**
* Prototypes declarations
*/
class Module;
class GraphicsManager;
class AudioManager;
class PythonManager;
class InputManager;
class SceneManager;
class SpriteManager;
class TextureManager;
class Editor;

enum class EngineModule
{
	GRAPHICS_MANAGER,
	AUDIO_MANAGER,
	INPUT_MANAGER,
	PYTHON_MANAGER,
	SCENE_MANAGER,
	EDITOR,
	LENGTH
};

/**
* \brief The main Engine class to centralise the frame process and the references
*/
class Engine
{
public:

	/**
	* \brief Initialize all the modules of the Game Engine, reading the config file too
	*/
	void Init(bool windowless=false, bool editor=false);
	/**
	* \brief Starting the Game Engine after the Init()
	*/
	void Start();

	/**
	 * \brief Destroy all the modules
	 */
	void Destroy();

	~Engine();
	/**
	* \brief A getter of the Configuration
	* \return The Configuration struct got by the Engine
	*/
	std::shared_ptr<Configuration> GetConfig();
	/**
	* \brief return the the pointer to an engine module
	* \parameter engineModule The Engine Module asked
	* \return the c++14 pointer of the module
	*/
	std::shared_ptr<Module> GetModule(EngineModule engineModule);

	bool running = false;
protected:
	std::shared_ptr<sf::RenderWindow> m_Window = nullptr;
	std::shared_ptr<Configuration> m_Config = nullptr;
	
	//modules
	std::array<std::shared_ptr<Module>, (int)EngineModule::LENGTH> modules;
	std::shared_ptr<GraphicsManager> m_GraphicsManager;
	std::shared_ptr<AudioManager> m_AudioManager;
	std::shared_ptr<SceneManager> m_SceneManager;
	std::shared_ptr<InputManager> m_InputManager;
	std::shared_ptr<PythonManager> m_PythonManager;
	std::shared_ptr<Editor> m_Editor;
};

/**
* \brief Module are classes used by the Engine to init and update features
*/
class Module
{
public:
	Module(Engine& engine, bool enable);

	virtual ~Module() {};
	/**
	* \brief Called to initialize the module
	*/
	virtual void Init() = 0;
	/**
	* \brief Called every frame to update the module
	* \param dt The delta time since last frame
	*/
	virtual void Update(sf::Time dt) = 0;
	/**
	* \brief Used instead of the destructor to delete all heap created structure and finalize
	*/
	virtual void Destroy() = 0;

	void SetEnable(bool enable);
	bool GetEnable();

protected:
	bool m_Enable = true;
	Engine& m_Engine;
};

}

#endif // !SFGE_ENGINE_H
