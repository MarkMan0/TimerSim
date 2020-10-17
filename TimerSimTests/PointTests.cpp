#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../TimerSim/Point.h"


namespace TimerSimTests {

	TEST_CLASS(PointTests) {

		using P = BasePoint<int>;

		TEST_METHOD(TestPlusMinus) {
			P p1(1, 1, 1), p2(2, 2, 2);
			P res;

			res = p1 + p2;
			Assert::AreEqual(3, res.x_, L"Addition wrong");
			Assert::AreEqual(3, res.y_, L"Addition wrong");
			Assert::AreEqual(3, res.z_, L"Addition wrong");

			res = p1 + p1;
			Assert::AreEqual(2, res.x_, L"Self Addition wrong");
			Assert::AreEqual(2, res.y_, L"Self Addition wrong");
			Assert::AreEqual(2, res.z_, L"Self Addition wrong");

			res = p1 - p2;
			Assert::AreEqual(-1, res.x_, L"Substraction wrong");
			Assert::AreEqual(-1, res.y_, L"Substraction wrong");
			Assert::AreEqual(-1, res.z_, L"Substraction wrong");
		}

		TEST_METHOD(TestComparision) {
			P p1(1, 1, 0), p2(1, 2, 0), p3(2, 2, 0), p4(1, 1, 0);
			Assert::IsFalse(p1 == p2);
			Assert::IsFalse(p1 == p3);
			Assert::IsFalse(p2 == p3);
			Assert::IsTrue(p1 == p4);
		}

	};


};