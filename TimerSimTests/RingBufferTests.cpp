#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../TimerSim/RingBuffer.h"


namespace TimerSimTests {

	TEST_CLASS(RingBufferTests)
	{
	private:
		static constexpr uint8_t BUFF_SZ = 4;
		using RB = RingBuffer<int, BUFF_SZ>;
		void insert(RB& buff, int val) {
			auto head = buff.getHead();
			*head = val;
			buff.commitHead();
		}
	public:

		TEST_METHOD(TestUtility)
		{
			RB buff;
			Assert::AreEqual(static_cast<uint8_t>(0), buff.size(), L"Size not 0 after creating");
			Assert::IsTrue(buff.hasFree(), L"hasFree was false after creating");
			Assert::IsFalse(buff.hasItems(), L"hasItems was true after creating");
			Assert::IsTrue(buff.isEmpty(), L"isEmpty was false after creating");
			Assert::IsFalse(buff.isFull(), L"isFull was true after creating");

			insert(buff, 1);
			insert(buff, 2);
			Assert::AreEqual(static_cast<uint8_t>(2), buff.size(), L"Size was not 2");
			Assert::IsTrue(buff.hasFree(), L"hasFree was false with 2 items");
			Assert::IsTrue(buff.hasItems(), L"hasItems was false with 2 items");
			Assert::IsFalse(buff.isEmpty(), L"isEmpty was true with 2 items");
			Assert::IsFalse(buff.isFull(), L"isFull was true with 2 items");

			buff.advanceTail();
			buff.advanceTail();
			Assert::AreEqual(static_cast<uint8_t>(0), buff.size(), L"Size was not 0");
			Assert::IsTrue(buff.hasFree(), L"hasFree was false on empty");
			Assert::IsFalse(buff.hasItems(), L"hasItems was true on empty");
			Assert::IsTrue(buff.isEmpty(), L"isEmpty was false on empty");
			Assert::IsFalse(buff.isFull(), L"isFull was true on empty");

			for (int i = 0; i < BUFF_SZ; ++i) {
				insert(buff, i);
			}
			Assert::AreEqual(BUFF_SZ, buff.size(), L"Size was not BUFF_SZ");
			Assert::IsFalse(buff.hasFree(), L"hasFree was false on full");
			Assert::IsTrue(buff.hasItems(), L"hasItems was true on full");
			Assert::IsFalse(buff.isEmpty(), L"isEmpty was false on full");
			Assert::IsTrue(buff.isFull(), L"isFull was true on full");
		}

		TEST_METHOD(TestWriteRead) {
			RB buff;

			for (int i = 0; i < BUFF_SZ; ++i) {
				insert(buff, i);
				Assert::AreEqual(static_cast<uint8_t>(i + 1), buff.size(), L"Insert failed");
			}
			Assert::IsTrue(buff.isFull(), L"Buffer wasn't full");

			for (int i = 0; i < BUFF_SZ; ++i) {
				auto tail = buff.getTail();
				Assert::AreEqual(i, *tail, L"Values don't match");
				buff.advanceTail();
				Assert::AreEqual(static_cast<uint8_t>(BUFF_SZ - i - 1), buff.size(), L"Size is wrong while reading");
			}
		}
	};

};