/**
 * Database Migration for Personalization Features
 * Adds tables for user preferences, profile, and skills
 */
import { config } from "dotenv";
import { Pool } from "pg";

// Load environment variables
config();

if (!process.env.DATABASE_URL) {
  console.error("❌ DATABASE_URL not found in environment variables");
  process.exit(1);
}

console.log("🔗 Connecting to database for personalization migration...");

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  },
});

const createPersonalizationTables = `
-- User Preferences Table
-- Stores theme, language, and notification preferences
CREATE TABLE IF NOT EXISTS "user_preferences" (
  id TEXT PRIMARY KEY DEFAULT gen_random_uuid()::text,
  "userId" TEXT NOT NULL UNIQUE REFERENCES "user"(id) ON DELETE CASCADE,
  theme TEXT DEFAULT 'system' CHECK (theme IN ('light', 'dark', 'system')),
  language TEXT DEFAULT 'en' CHECK (language IN ('en', 'ur')),
  "chatHistoryEnabled" BOOLEAN DEFAULT TRUE,
  "emailNotifications" BOOLEAN DEFAULT TRUE,
  "chatNotifications" BOOLEAN DEFAULT TRUE,
  "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User Profile Table
-- Stores extended profile information
CREATE TABLE IF NOT EXISTS "user_profile" (
  id TEXT PRIMARY KEY DEFAULT gen_random_uuid()::text,
  "userId" TEXT NOT NULL UNIQUE REFERENCES "user"(id) ON DELETE CASCADE,
  bio TEXT,
  "displayName" TEXT,
  "phoneNumber" TEXT,
  location TEXT,
  website TEXT,
  "socialLinks" JSONB DEFAULT '{}'::jsonb,
  "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User Skills Table
-- Stores programming skills and experience data from EnhancedSignup
CREATE TABLE IF NOT EXISTS "user_skills" (
  id TEXT PRIMARY KEY DEFAULT gen_random_uuid()::text,
  "userId" TEXT NOT NULL UNIQUE REFERENCES "user"(id) ON DELETE CASCADE,
  "softwareExperience" TEXT CHECK ("softwareExperience" IN ('beginner', 'intermediate', 'advanced', 'none')),
  "hardwareExperience" TEXT CHECK ("hardwareExperience" IN ('beginner', 'intermediate', 'advanced', 'none')),
  "programmingLevel" TEXT CHECK ("programmingLevel" IN ('beginner', 'intermediate', 'advanced', 'expert')),
  "programmingLanguages" TEXT[] DEFAULT ARRAY[]::TEXT[],
  "learningGoals" TEXT,
  "industryBackground" TEXT,
  "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User Activity Table (Optional - for tracking user engagement)
CREATE TABLE IF NOT EXISTS "user_activity" (
  id TEXT PRIMARY KEY DEFAULT gen_random_uuid()::text,
  "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
  "activityType" TEXT NOT NULL,
  "activityData" JSONB DEFAULT '{}'::jsonb,
  "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for better query performance
CREATE INDEX IF NOT EXISTS idx_user_preferences_user_id ON "user_preferences"("userId");
CREATE INDEX IF NOT EXISTS idx_user_profile_user_id ON "user_profile"("userId");
CREATE INDEX IF NOT EXISTS idx_user_skills_user_id ON "user_skills"("userId");
CREATE INDEX IF NOT EXISTS idx_user_activity_user_id ON "user_activity"("userId");
CREATE INDEX IF NOT EXISTS idx_user_activity_created_at ON "user_activity"("createdAt" DESC);
`;

export async function runPersonalizationMigration() {
  try {
    console.log("🔄 Running personalization migrations...");

    await pool.query(createPersonalizationTables);

    console.log("✅ Personalization migrations completed successfully!");
    console.log("📊 Tables created:");
    console.log("   - user_preferences (theme, language, notifications)");
    console.log("   - user_profile (bio, display name, social links)");
    console.log("   - user_skills (programming experience, languages)");
    console.log("   - user_activity (activity tracking)");

    await pool.end();
  } catch (error) {
    console.error("❌ Personalization migration failed:", error);
    process.exit(1);
  }
}

// Run migration if this file is executed directly
if (require.main === module) {
  runPersonalizationMigration();
}
