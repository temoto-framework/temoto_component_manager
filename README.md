# temoto_component_manager

[More information and tutorials](https://github.com/temoto-framework/temoto/wiki).

Compnent Manager is a ROS node, which maintains information about components (including published/subscribed topics, package names) and can dynamically compose them based on component templates. Components are ROS based programs (nodes/launch files) which provide sensing or data processing functionalities. A component template outlines the structure of a composed component, e.g., a combination of a manipulator arm and a force-torque sensor with compliance controller. Templates can then be used to dynamically combine individual components and replace faulty ones.
