/**
 * Simple Express Server for Chatbot (without database dependency)
 * Handles chat API without authentication
 */
import express from "express";
import cors from "cors";

const app = express();
const PORT = process.env.PORT || 3001;

// Middleware
app.use(cors({
  origin: process.env.CLIENT_URL || "http://localhost:3000",
  credentials: true,
}));
app.use(express.json());

// Health check endpoint
app.get("/api/health", (req, res) => {
  res.json({ status: "ok", message: "Chat server is running" });
});

// Chat endpoint for the chatbot
app.post("/api/chat", async (req, res) => {
  try {
    const { message, target_language, selected_text } = req.body;

    console.log("💬 Chat request received:", { message, target_language, selected_text });

    // Check if OpenAI API key is configured
    const OPENAI_API_KEY = process.env.OPENAI_API_KEY;

    if (!OPENAI_API_KEY) {
      console.log("⚠️ OPENAI_API_KEY not configured, using fallback response");

      const fallbackResponse = target_language === 'ur'
        ? `مجھے آپ کا سوال "${message}" موصول ہوا۔ میں ابھی AI سے منسلک نہیں ہوں، لیکن میں آپ کی مدد کے لیے تیار ہوں۔`
        : `I received your question "${message}". I'm not currently connected to AI, but I'm here to help you.`;

      return res.json({
        response: fallbackResponse,
        language: target_language || 'en'  // Include language in response
      });
    }

    // Prepare the prompt
    const systemPrompt = target_language === 'ur'
      ? "آپ ایک مددگار روبوٹکس اور AI اسسٹنٹ ہیں۔ صارف کے سوالات کا اردو میں جواب دیں۔"
      : "You are a helpful robotics and AI assistant. Provide clear and informative responses about robotics, AI, and related topics.";

    const userPrompt = selected_text
      ? `Context: The user selected this text: "${selected_text}"\n\nUser question: ${message}`
      : message;

    // Call OpenAI API
    const response = await fetch(
      "https://api.openai.com/v1/chat/completions",
      {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "Authorization": `Bearer ${OPENAI_API_KEY}`,
        },
        body: JSON.stringify({
          model: "gpt-3.5-turbo",
          messages: [
            {
              role: "system",
              content: systemPrompt
            },
            {
              role: "user",
              content: userPrompt
            }
          ],
          temperature: 0.7,
          max_tokens: 1000,
        }),
      }
    );

    if (!response.ok) {
      const errorData = await response.text();
      console.error("OpenAI API error:", response.status, errorData);
      throw new Error(`OpenAI API error: ${response.status}`);
    }

    const data = await response.json() as { choices?: { message?: { content: string } }[] };
    const aiResponse = data.choices?.[0]?.message?.content || "No response generated";

    console.log("✅ Chat response sent");
    res.json({
      response: aiResponse,
      language: req.body.target_language || 'en'  // Include the requested language
    });

  } catch (error) {
    console.error("❌ Chat error:", error);

    const errorMessage = req.body.target_language === 'ur'
      ? 'معذرت، کوئی خرابی ہوئی۔ براہ کرم دوبارہ کوشش کریں۔'
      : 'Sorry, an error occurred. Please try again.';

    res.status(500).json({
      response: errorMessage,
      language: req.body.target_language || 'en', // Include language in error response too
      error: process.env.NODE_ENV === 'development' ? String(error) : undefined
    });
  }
});

// Error handling middleware
app.use((err: any, req: express.Request, res: express.Response, next: express.NextFunction) => {
  console.error("Server error:", err);
  res.status(500).json({
    error: "Internal server error",
    message: process.env.NODE_ENV === 'development' ? err.message : undefined
  });
});

// Start server
app.listen(PORT, () => {
  console.log(`🚀 Chat server running on port ${PORT}`);
  console.log(`📍 Health check: http://localhost:${PORT}/api/health`);
  console.log(`📍 Chat endpoint: http://localhost:${PORT}/api/chat`);
});

export default app;