function hello() {
  return "hello";
}

test("says hello", () => {
  expect(hello()).toBe("hello");
});
