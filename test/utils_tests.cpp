#include "../src/module/utils.hpp"
#include <atomic>
#include <gtest/gtest.h>

// Mock C-style cleanup functions for testing
namespace {

// Counter to track cleanup calls
std::atomic<int> cleanup_call_count{0};
std::atomic<int> cleanup_with_error_call_count{0};

// Simple cleanup function that takes a pointer
void mock_cleanup(int *ptr) {
  if (ptr) {
    cleanup_call_count++;
  }
}

// Cleanup function with error code parameter
void mock_cleanup_with_error(int *ptr, int *error_out) {
  if (ptr) {
    cleanup_with_error_call_count++;
    if (error_out) {
      *error_out = 0; // Success
    }
  }
}

// Helper to create a resource
int *create_resource() { return new int(42); }

} // anonymous namespace

class CleanupPtrTest : public ::testing::Test {
protected:
  void SetUp() override {
    cleanup_call_count = 0;
    cleanup_with_error_call_count = 0;
  }
};

TEST_F(CleanupPtrTest, BasicCleanup_CallsCleanupOnDestruction) {
  {
    utils::CleanupPtr<mock_cleanup> ptr(create_resource());
    EXPECT_NE(ptr.get(), nullptr);
    EXPECT_EQ(cleanup_call_count, 0);
  }
  // Cleanup should be called when ptr goes out of scope
  EXPECT_EQ(cleanup_call_count, 1);
}

TEST_F(CleanupPtrTest, NullPointer_DoesNotCallCleanup) {
  {
    utils::CleanupPtr<mock_cleanup> ptr(nullptr);
    EXPECT_EQ(ptr.get(), nullptr);
  }
  // Cleanup should not be called for null pointer
  EXPECT_EQ(cleanup_call_count, 0);
}

TEST_F(CleanupPtrTest, MoveConstructor_TransfersOwnership) {
  utils::CleanupPtr<mock_cleanup> ptr1(create_resource());
  int *raw_ptr = ptr1.get();

  utils::CleanupPtr<mock_cleanup> ptr2(std::move(ptr1));

  EXPECT_EQ(ptr1.get(), nullptr);   // Original is now empty
  EXPECT_EQ(ptr2.get(), raw_ptr);   // New pointer owns the resource
  EXPECT_EQ(cleanup_call_count, 0); // Not cleaned up yet

  ptr2.reset();
  EXPECT_EQ(cleanup_call_count, 1); // Cleaned up once
}

TEST_F(CleanupPtrTest, MoveAssignment_TransfersOwnership) {
  utils::CleanupPtr<mock_cleanup> ptr1(create_resource());
  int *raw_ptr = ptr1.get();

  utils::CleanupPtr<mock_cleanup> ptr2(create_resource());
  EXPECT_EQ(cleanup_call_count, 0);

  ptr2 = std::move(ptr1);

  // ptr2's old resource should be cleaned up
  EXPECT_EQ(cleanup_call_count, 1);
  EXPECT_EQ(ptr1.get(), nullptr);
  EXPECT_EQ(ptr2.get(), raw_ptr);
}

TEST_F(CleanupPtrTest, Reset_CleansUpAndAcceptsNewPointer) {
  utils::CleanupPtr<mock_cleanup> ptr(create_resource());
  EXPECT_NE(ptr.get(), nullptr);

  int *new_resource = create_resource();
  ptr.reset(new_resource);

  EXPECT_EQ(cleanup_call_count, 1); // Old resource cleaned up
  EXPECT_EQ(ptr.get(), new_resource);

  ptr.reset();
  EXPECT_EQ(cleanup_call_count, 2); // New resource cleaned up
  EXPECT_EQ(ptr.get(), nullptr);
}

TEST_F(CleanupPtrTest, Release_ReturnsPointerWithoutCleanup) {
  utils::CleanupPtr<mock_cleanup> ptr(create_resource());
  int *raw_ptr = ptr.get();

  int *released = ptr.release();

  EXPECT_EQ(released, raw_ptr);
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_EQ(cleanup_call_count, 0); // Not cleaned up

  // Manual cleanup required
  delete released;
}

TEST_F(CleanupPtrTest, BoolConversion_CorrectlyIndicatesOwnership) {
  utils::CleanupPtr<mock_cleanup> ptr1(create_resource());
  EXPECT_TRUE(ptr1);
  EXPECT_TRUE(static_cast<bool>(ptr1));

  utils::CleanupPtr<mock_cleanup> ptr2(nullptr);
  EXPECT_FALSE(ptr2);
  EXPECT_FALSE(static_cast<bool>(ptr2));
}

TEST_F(CleanupPtrTest, Dereference_AccessesUnderlyingValue) {
  int *resource = new int(123);
  utils::CleanupPtr<mock_cleanup> ptr(resource);

  EXPECT_EQ(*ptr, 123);
  *ptr = 456;
  EXPECT_EQ(*ptr, 456);
}

TEST_F(CleanupPtrTest, ExceptionSafety_CleansUpOnException) {
  try {
    utils::CleanupPtr<mock_cleanup> ptr(create_resource());
    EXPECT_EQ(cleanup_call_count, 0);
    throw std::runtime_error("Test exception");
  } catch (const std::runtime_error &) {
    // Exception caught
  }

  // Resource should still be cleaned up despite exception
  EXPECT_EQ(cleanup_call_count, 1);
}

TEST_F(CleanupPtrTest, MultipleInstances_EachCleansUpIndependently) {
  {
    utils::CleanupPtr<mock_cleanup> ptr1(create_resource());
    utils::CleanupPtr<mock_cleanup> ptr2(create_resource());
    utils::CleanupPtr<mock_cleanup> ptr3(create_resource());

    EXPECT_EQ(cleanup_call_count, 0);
  }

  EXPECT_EQ(cleanup_call_count, 3);
}

// Test that CleanupPtr properly handles the cleanup count
TEST_F(CleanupPtrTest, Integration_VerifiesCleanupOrder) {
  int first_count = 0;
  int second_count = 0;

  {
    utils::CleanupPtr<mock_cleanup> ptr1(create_resource());
    first_count = cleanup_call_count;
    {
      utils::CleanupPtr<mock_cleanup> ptr2(create_resource());
      EXPECT_EQ(cleanup_call_count, 0);
    }
    second_count = cleanup_call_count;
  }

  EXPECT_EQ(first_count, 0);
  EXPECT_EQ(second_count, 1);
  EXPECT_EQ(cleanup_call_count, 2);
}

TEST(ContainsTest, ElementPresent_ReturnsTrue) {
  std::vector<int> vec = {1, 2, 3, 4, 5};
  EXPECT_TRUE(utils::contains(3, vec));
  EXPECT_TRUE(utils::contains(1, vec));
  EXPECT_TRUE(utils::contains(5, vec));
}

TEST(ContainsTest, ElementAbsent_ReturnsFalse) {
  std::vector<int> vec = {1, 2, 3, 4, 5};
  EXPECT_FALSE(utils::contains(0, vec));
  EXPECT_FALSE(utils::contains(6, vec));
  EXPECT_FALSE(utils::contains(10, vec));
}

TEST(ContainsTest, EmptyContainer_ReturnsFalse) {
  std::vector<int> vec;
  EXPECT_FALSE(utils::contains(1, vec));
}

TEST(ContainsTest, StringContainer_WorksCorrectly) {
  std::vector<std::string> vec = {"apple", "banana", "cherry"};
  EXPECT_TRUE(utils::contains(std::string("banana"), vec));
  EXPECT_FALSE(utils::contains(std::string("grape"), vec));
}
