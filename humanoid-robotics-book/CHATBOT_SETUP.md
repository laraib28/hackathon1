# Chatbot Setup Guide

The chatbot has been integrated with OpenAI API and is now ready to use!

## ✅ What's Done

- Chat endpoint added to Better Auth server (`/api/chat`)
- OpenAI GPT-3.5-turbo integration
- Support for both English and Urdu responses
- Context-aware responses (can explain selected text)

## 🚀 Setup Steps

### Step 1: Add Your OpenAI API Key

Edit `server/.env` and add your OpenAI API key:

```env
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

**Get your API key:**
1. Go to https://platform.openai.com/api-keys
2. Login to your OpenAI account
3. Click "Create new secret key"
4. Copy the key and paste it in `server/.env`

### Step 2: Start the Server

If the server is already running, restart it:

```bash
# Stop the current server (Ctrl+C)
# Then restart:
npm run server:dev
```

Or use the automated script:

```bash
./start-dev.sh
```

### Step 3: Test the Chatbot

1. Open http://localhost:3000
2. Click the chat button (💬) in the bottom right
3. Type a question in English or Urdu
4. Get AI-powered responses!

**Example questions:**
- "What is ROS 2?"
- "روبوٹکس کیا ہے؟" (What is robotics?)
- "Explain digital twin"
- "مجھے AI کے بارے میں بتائیں" (Tell me about AI)

## 🔧 How It Works

### Chat Endpoint

```
POST /api/chat
```

**Request:**
```json
{
  "message": "What is ROS 2?",
  "target_language": "en",
  "selected_text": null
}
```

**Response:**
```json
{
  "response": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software..."
}
```

### Features

1. **Language Detection**: Automatically detects if user is writing in Urdu or English
2. **Context-Aware**: Can explain selected text from the book
3. **Fallback Mode**: Works even without API key (returns friendly message)
4. **Error Handling**: Graceful error messages in both languages

### OpenAI Configuration

- **Model**: GPT-3.5-turbo (fast and cost-effective)
- **Temperature**: 0.7 (balanced creativity)
- **Max Tokens**: 1000 (sufficient for detailed responses)
- **System Prompt**: Configured for robotics and AI assistance

## 💰 OpenAI Pricing

GPT-3.5-turbo is very affordable:
- **Input**: $0.0005 per 1K tokens (~$0.50 per million tokens)
- **Output**: $0.0015 per 1K tokens (~$1.50 per million tokens)

**Example costs:**
- 100 questions = ~$0.05
- 1,000 questions = ~$0.50
- 10,000 questions = ~$5.00

## 🔒 Security

- API key is stored in `.env` (not committed to git)
- Server-side API calls (key never exposed to frontend)
- CORS protection
- Request validation

## 🛠️ Troubleshooting

### "Server is not responding"

**Check:**
1. Is the server running? `npm run server:dev`
2. Is `OPENAI_API_KEY` set in `server/.env`?
3. Check server console for errors

**Test the endpoint directly:**
```bash
curl -X POST http://localhost:3001/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"Hello","target_language":"en"}'
```

### "OpenAI API error"

**Common causes:**
1. Invalid API key
2. Insufficient credits in OpenAI account
3. Rate limit exceeded

**Solutions:**
1. Verify API key at https://platform.openai.com/api-keys
2. Add credits to your OpenAI account
3. Wait a few minutes and try again

### Chatbot shows but doesn't respond

**Check browser console (F12):**
- Look for CORS errors
- Check if requests are reaching the server
- Verify `REACT_APP_API_URL` in `.env.local`

**Verify `.env.local`:**
```env
REACT_APP_API_URL=http://localhost:3001
```

## 🌍 Language Support

### English
- Automatically responds in English for English questions
- System prompt: "You are a helpful robotics and AI assistant..."

### Urdu (اردو)
- Automatically responds in Urdu for Urdu questions
- System prompt: "آپ ایک مددگار روبوٹکس اور AI اسسٹنٹ ہیں..."

### How to switch?

The chatbot automatically detects the language based on Unicode characters:
- Contains Urdu characters (U+0600-U+06FF) → Urdu response
- Otherwise → English response

## 🚀 Production Deployment

When deploying to production:

1. **Set environment variable on Railway/Vercel:**
   ```
   OPENAI_API_KEY=sk-proj-your-production-key
   ```

2. **Update `.env.production`:**
   ```env
   REACT_APP_API_URL=https://your-backend.railway.app
   ```

3. **Monitor usage:**
   - Check OpenAI dashboard for API usage
   - Set up usage alerts in OpenAI dashboard
   - Consider rate limiting for production

## 📊 Monitoring

Check server logs to see chat requests:

```
💬 Chat request received: { message: 'What is ROS 2?', target_language: 'en' }
✅ Chat response sent
```

## 🎯 Next Steps

- ✅ Chatbot is working with OpenAI
- ✅ Supports English and Urdu
- ✅ Can explain selected text
- Optional: Add conversation history
- Optional: Add typing indicator improvements
- Optional: Add rate limiting
- Optional: Add user feedback mechanism

## 📚 Resources

- **OpenAI API Docs**: https://platform.openai.com/docs
- **API Keys**: https://platform.openai.com/api-keys
- **Usage Dashboard**: https://platform.openai.com/usage
- **Pricing**: https://openai.com/api/pricing/

Happy chatting! 🤖💬
