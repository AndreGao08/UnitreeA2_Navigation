const form = document.querySelector("#loginForm");
const errorBox = document.querySelector("#loginError");
form.addEventListener("submit", async (event) => {
  event.preventDefault(); errorBox.textContent = "";
  const values = new FormData(form);
  try {
    const response = await fetch("/api/auth/login", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ username: values.get("username"), password: values.get("password") }),
    });
    if (!response.ok) throw new Error((await response.json()).detail || "登录失败");
    window.location.replace("/");
  } catch (error) { errorBox.textContent = error.message; }
});
