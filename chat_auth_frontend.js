// Save Better Auth token to localStorage
function saveAuthToken(token) {
  localStorage.setItem('auth_token', token);
}

// Function to make chat API calls with auth header
async function chatAPI(message) {
  const token = localStorage.getItem('auth_token');
  const response = await fetch('/api/chat', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`
    },
    body: JSON.stringify({ message: message })
  });
  return response.json();
}