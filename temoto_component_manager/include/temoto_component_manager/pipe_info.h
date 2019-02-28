#ifndef TEMOTO_COMPONENT_MANAGER__PIPE_INFO_H
#define TEMOTO_COMPONENT_MANAGER__PIPE_INFO_H

#include "temoto_core/common/temoto_log_macros.h"
#include "temoto_core/common/reliability.h"

#include <string>
#include <vector>
#include <map>
#include <set>
#include <ctype.h>
#include <memory>                     // shared_ptr
#include <yaml-cpp/yaml.h>
#include <iostream>                   // TODO: remove

namespace temoto_component_manager
{

/**
 * @brief The SegmentCategory enum. TODO: DEPRECATED
 */
enum class SegmentCategory : int
{
  SENSOR,
  ALGORITHM
};

/**
 * @brief The Segment struct
 */
struct Segment
{
  std::string segment_category_;                       // Sensor or algorithm or ... TODO: DEPRECATED
  std::string segment_type_;                           // Camera ... or ARtag detector ...
  std::set<std::string> required_input_topic_types_;  // The types of the topics that this segment requires
  std::set<std::string> required_output_topic_types_; // The types of the topics that this segment must publish

  /// add input topic type
  void addInputTopicType(std::string topic_type)
  {
    required_input_topic_types_.insert(topic_type);
  }

  /// add output topic type
  void addOutputTopicType(std::string topic_type)
  {
    required_output_topic_types_.insert(topic_type);
  }

  /// to string
  std::string toString() const
  {
    std::string str;
    str += "|_+_segment category: " + segment_category_ + "\n";
    str += "| |_segment type: " + segment_type_ + "\n";

    // Print out the input topics
    if (!required_input_topic_types_.empty())
    {
      str += "| |_required input topic types: ";
      for (auto& topic : required_input_topic_types_)
      {
        str += topic;
        if (topic != *std::prev(required_input_topic_types_.end()))
        {
          str += ", ";
        }
      }
      str += "\n";
    }

    // Print out the output topics
    if (!required_output_topic_types_.empty())
    {
      str += "| |_required output topic types: ";
      for (auto& topic : required_output_topic_types_)
      {
        str += topic;
        if (topic != *std::prev(required_output_topic_types_.end()))
        {
          str += ", ";
        }
      }
      str += "\n";
    }

    return str;
  }
};

/**
 * @brief operator ==
 * @param f1
 * @param f2
 * @return
 */
static bool operator==(const Segment& f1, const Segment& f2)
{
  // Check the category, type and topic types
  return f1.segment_category_ == f2.segment_category_ &&
         f1.segment_type_ == f2.segment_type_ &&
         f1.required_input_topic_types_ == f2.required_input_topic_types_ &&
         f1.required_output_topic_types_ == f2.required_output_topic_types_;
}

/**
 * @brief operator <<
 * @param out
 * @param f
 * @return
 */
static std::ostream& operator<<(std::ostream& out, const Segment& f)
{
    out << f.toString();

    return out;
}

/**
 * @brief The PipeInfo class
 */
class PipeInfo
{
public:

  temoto_core::Reliability reliability_;

  /*
   * Getters
   */

  /// Get type
  std::string getType() const
  {
    return type_;
  }

  /// Get pipe
  std::vector<Segment> getPipe() const
  {
    return pipe_;
  }

  /// Get pipe size
  unsigned int getPipeSize() const
  {
    return pipe_.size();
  }

  /*
   * Setters
   */

  /// Set the pipe
  void setPipe(std::vector<Segment> pipe)
  {
    pipe_ = pipe;
  }

  void setType(std::string type)
  {
    type_ = type;
  }

  /// Add segment
  void addSegment(Segment segment)
  {
    pipe_.push_back(segment);
  }

  std::string toString()
  {
    std::string str;
    str += "type: " + std::string("TODO") + "\n";
    str += "reliability: " + std::to_string(reliability_.getReliability()) + "\n";

    for (auto& segment : pipe_)
    {
      str += segment.toString();

      if (&segment != &pipe_.back())
      {
        str += "| \n";
      }
    }

    return str;
  }

  /**
   * @brief operator ==
   * @param t1
   * @param t2
   * @return
   */
  friend bool operator==(const PipeInfo& t1, const PipeInfo& t2)
  {
    return t1.type_ == t2.type_ && t1.pipe_ == t2.pipe_;
  }

private:
  
  std::string type_;
  std::vector<Segment> pipe_;
};


/**
 * @brief PipeInfoPtr
 */
typedef std::shared_ptr<PipeInfo> PipeInfoPtr;

/**
 * @brief PipeInfoPtrs
 */
typedef std::vector<PipeInfoPtr> PipeInfoPtrs;

} // namespace temoto_component_manager

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                      YAML PARSER FOR TRACKER INFO CLASS
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

namespace YAML
{
template <>
struct convert<temoto_component_manager::PipeInfo>
{
  static Node encode(const temoto_component_manager::PipeInfo& pipe_info)
  {
    Node method;
    std::vector<temoto_component_manager::Segment> pipe = pipe_info.getPipe();
    for (auto& segment : pipe)
    {
      // Encode the segment
      Node segment_node;
      segment_node["segment_category"] = segment.segment_category_;
      segment_node["segment_type"] = segment.segment_type_;

      // Encode the input topic types (if this segment has any)
      if (segment.required_input_topic_types_.size() != 0)
      {
        for (auto& topic_type : segment.required_input_topic_types_)
        {
          segment_node["input_topic_types"].push_back(topic_type);
        }
      }

      // Encode the output topic types (if this segment has any)
      if (segment.required_output_topic_types_.size() != 0)
      {
        for (auto& topic_type : segment.required_output_topic_types_)
        {
          segment_node["output_topic_types"].push_back(topic_type);
        }
      }

      // Push the segment
      method.push_back(segment_node);
    }

    return method;
  }

  static bool decode(const Node& node, temoto_component_manager::PipeInfo& pipe_info)
  {
    // Check if the "node" is a map
    if (!node.IsMap())
    {
      return false;
    }

    // Get the pipe (sequence of segments) node
    YAML::Node segments_node = node["method"];

    // Iterate over each segment
    for (YAML::const_iterator segment_it = segments_node.begin(); segment_it != segments_node.end(); ++segment_it)
    {
      // Check if the segment is a map
      if (!segment_it->IsMap())
      {
        return false;
      }

      // Create an empty segment object and fill it
      temoto_component_manager::Segment segment;

      try
      {
        // TODO: Check if it is even a valid category
        segment.segment_category_ = (*segment_it)["segment_category"].as<std::string>();
        segment.segment_type_ = (*segment_it)["segment_type"].as<std::string>();
      }
      catch (YAML::InvalidNode e)
      {
        // Print out the error message
        // TODO: throw a proper error
        std::cout << "The segment node is either missing category or type\n";
        return false;
      }

      // Get the input topic types (if there are any)
      try
      {
        Node input_topics_node = (*segment_it)["input_topic_types"];
        for (YAML::const_iterator topics_it = input_topics_node.begin(); topics_it != input_topics_node.end(); ++topics_it)
        {
          segment.addInputTopicType(topics_it->as<std::string>());
        }
      }
      catch (YAML::InvalidNode e)
      {
        // TODO: REPORT OR DO SOMETHING
      }

      // Get the output topic types (if there are any)
      try
      {
        Node output_topics_node = (*segment_it)["output_topic_types"];
        for (YAML::const_iterator topics_it = output_topics_node.begin(); topics_it != output_topics_node.end(); ++topics_it)
        {
          segment.addOutputTopicType(topics_it->as<std::string>());
        }
      }
      catch (YAML::InvalidNode e)
      {
        // REPORT OR DO SOMETHING
      }

      pipe_info.addSegment(segment);
    }

    return true;
  }
};
}
#endif
