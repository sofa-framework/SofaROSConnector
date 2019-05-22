// Generated by gencpp from file zyrosconnector_test/ArrayOfFloats.msg
// DO NOT EDIT!


#ifndef ZYROSCONNECTOR_TEST_MESSAGE_ARRAYOFFLOATS_H
#define ZYROSCONNECTOR_TEST_MESSAGE_ARRAYOFFLOATS_H

#include <ros/service_traits.h>


#include <zyrosconnector_test/ArrayOfFloatsRequest.h>
#include <zyrosconnector_test/ArrayOfFloatsResponse.h>


namespace zyrosconnector_test
{

struct ArrayOfFloats
{

typedef ArrayOfFloatsRequest Request;
typedef ArrayOfFloatsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ArrayOfFloats
} // namespace zyrosconnector_test


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::zyrosconnector_test::ArrayOfFloats > {
  static const char* value()
  {
    return "c5747b06dc5a2cb86a33a7acc3e77cde";
  }

  static const char* value(const ::zyrosconnector_test::ArrayOfFloats&) { return value(); }
};

template<>
struct DataType< ::zyrosconnector_test::ArrayOfFloats > {
  static const char* value()
  {
    return "zyrosconnector_test/ArrayOfFloats";
  }

  static const char* value(const ::zyrosconnector_test::ArrayOfFloats&) { return value(); }
};


// service_traits::MD5Sum< ::zyrosconnector_test::ArrayOfFloatsRequest> should match 
// service_traits::MD5Sum< ::zyrosconnector_test::ArrayOfFloats > 
template<>
struct MD5Sum< ::zyrosconnector_test::ArrayOfFloatsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::zyrosconnector_test::ArrayOfFloats >::value();
  }
  static const char* value(const ::zyrosconnector_test::ArrayOfFloatsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::zyrosconnector_test::ArrayOfFloatsRequest> should match 
// service_traits::DataType< ::zyrosconnector_test::ArrayOfFloats > 
template<>
struct DataType< ::zyrosconnector_test::ArrayOfFloatsRequest>
{
  static const char* value()
  {
    return DataType< ::zyrosconnector_test::ArrayOfFloats >::value();
  }
  static const char* value(const ::zyrosconnector_test::ArrayOfFloatsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::zyrosconnector_test::ArrayOfFloatsResponse> should match 
// service_traits::MD5Sum< ::zyrosconnector_test::ArrayOfFloats > 
template<>
struct MD5Sum< ::zyrosconnector_test::ArrayOfFloatsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::zyrosconnector_test::ArrayOfFloats >::value();
  }
  static const char* value(const ::zyrosconnector_test::ArrayOfFloatsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::zyrosconnector_test::ArrayOfFloatsResponse> should match 
// service_traits::DataType< ::zyrosconnector_test::ArrayOfFloats > 
template<>
struct DataType< ::zyrosconnector_test::ArrayOfFloatsResponse>
{
  static const char* value()
  {
    return DataType< ::zyrosconnector_test::ArrayOfFloats >::value();
  }
  static const char* value(const ::zyrosconnector_test::ArrayOfFloatsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ZYROSCONNECTOR_TEST_MESSAGE_ARRAYOFFLOATS_H
