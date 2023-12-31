#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

using namespace BT;

// Example of custom SyncActionNode (synchronous action)
// without ports.
class TaskStart : public BT::SyncActionNode
{
public:
    TaskStart(const std::string& name) :
        BT::SyncActionNode(name, {})
    {}

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "Starting execution of tasks!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class TaskEnd : public BT::SyncActionNode
{
public:
    TaskEnd(const std::string& name) :
        BT::SyncActionNode(name, {})
    {}

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "Task execution ended successfully!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class TaskA : public BT::StatefulActionNode
{
public:
    TaskA(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return{ BT::InputPort<int>("msec") };
    }

    NodeStatus onStart() override
    {
      int msec = 0;
      getInput("msec", msec);

      std::cout << "Executing task A" << std::endl;
      
      if( msec <= 0 ) {
        // No need to go into the RUNNING state
        std::cout << "Finished task A" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        // once the deadline is reached, we will return SUCCESS.
        deadline_ = std::chrono::system_clock::now() + std::chrono::milliseconds(msec);
        return NodeStatus::RUNNING;
      }
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      if ( std::chrono::system_clock::now() >= deadline_ ) {
        std::cout << "Finished task A" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "Task A interrupted" << std::endl;
    }

private:
    std::chrono::system_clock::time_point deadline_;
};

class TaskB : public BT::StatefulActionNode
{
public:
    TaskB(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return{ BT::InputPort<int>("msec") };
    }

    NodeStatus onStart() override
    {
      int msec = 0;
      getInput("msec", msec);

      std::cout << "Executing task B" << std::endl;
      
      if( msec <= 0 ) {
        // No need to go into the RUNNING state
        std::cout << "Finished task B" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        // once the deadline is reached, we will return SUCCESS.
        deadline_ = std::chrono::system_clock::now() + std::chrono::milliseconds(msec);
        return NodeStatus::RUNNING;
      }
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      if ( std::chrono::system_clock::now() >= deadline_ ) {
        std::cout << "Finished task B" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "Task B interrupted" << std::endl;
    }

private:
    std::chrono::system_clock::time_point deadline_;
};

class TaskC : public BT::StatefulActionNode
{
public:
    TaskC(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return{ BT::InputPort<int>("msec") };
    }

    NodeStatus onStart() override
    {
      int msec = 0;
      getInput("msec", msec);

      std::cout << "Executing task C" << std::endl;
      
      if( msec <= 0 ) {
        // No need to go into the RUNNING state
        std::cout << "Finished task C" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        // once the deadline is reached, we will return SUCCESS.
        deadline_ = std::chrono::system_clock::now() + std::chrono::milliseconds(msec);
        return NodeStatus::RUNNING;
      }
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      if ( std::chrono::system_clock::now() >= deadline_ ) {
        std::cout << "Finished task C" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "Task C interrupted" << std::endl;
    }

private:
    std::chrono::system_clock::time_point deadline_;
};

class TaskD : public BT::StatefulActionNode
{
public:
    TaskD(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return{ BT::InputPort<int>("msec") };
    }

    NodeStatus onStart() override
    {
      int msec = 0;
      getInput("msec", msec);

      std::cout << "Executing task D" << std::endl;
      
      if( msec <= 0 ) {
        // No need to go into the RUNNING state
        std::cout << "Finished task D" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        // once the deadline is reached, we will return SUCCESS.
        deadline_ = std::chrono::system_clock::now() + std::chrono::milliseconds(msec);
        return NodeStatus::RUNNING;
      }
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      if ( std::chrono::system_clock::now() >= deadline_ ) {
        std::cout << "Finished task D" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "Task D interrupted" << std::endl;
    }

private:
    std::chrono::system_clock::time_point deadline_;
};

class TaskE : public BT::StatefulActionNode
{
public:
    TaskE(const std::string& name, const BT::NodeConfig& config) :
        BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return{ BT::InputPort<int>("msec") };
    }

    NodeStatus onStart() override
    {
      int msec = 0;
      getInput("msec", msec);

      std::cout << "Executing task E" << std::endl;
      
      if( msec <= 0 ) {
        // No need to go into the RUNNING state
        std::cout << "Finished task E" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        // once the deadline is reached, we will return SUCCESS.
        deadline_ = std::chrono::system_clock::now() + std::chrono::milliseconds(msec);
        return NodeStatus::RUNNING;
      }
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
      if ( std::chrono::system_clock::now() >= deadline_ ) {
        std::cout << "Finished task E" << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "Task E interrupted" << std::endl;
    }

private:
    std::chrono::system_clock::time_point deadline_;
};

void createFile(BT::BehaviorTreeFactory & factory){
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    
    // File name
    const std::string file_name = "./install/btcpp_sample/lib/btcpp_sample/bt_xml/tree_models.xml";

    // Open a file for writing
    std::ofstream file(file_name);

    // Check if the file is open
    if (file.is_open()) {
        // Write the string to the file
        file << xml_models;

        // Close the file
        file.close();

        std::cout << "String successfully saved to " << file_name << std::endl;
    } else {
        std::cerr << "Error opening the file " << file_name << std::endl;
    }
}

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<TaskStart>("TaskStart");
    factory.registerNodeType<TaskA>("TaskA");
    factory.registerNodeType<TaskB>("TaskB");
    factory.registerNodeType<TaskC>("TaskC");
    factory.registerNodeType<TaskD>("TaskD");
    factory.registerNodeType<TaskE>("TaskE");
    factory.registerNodeType<TaskEnd>("TaskEnd");

    // createFile(factory);

    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 
        
    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("./install/behavior_trees/lib/behavior_trees/bt_xml/my_tree.xml");

    // Connect with Groot2
    BT::Groot2Publisher publisher(tree);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickWhileRunning();

    return 0;
}