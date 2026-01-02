/**
 * Personalization API Endpoints
 * Handles user profile, preferences, and skills management
 */
import { Router, Request, Response } from 'express';
import { Pool } from 'pg';
import { config } from 'dotenv';

config();

const router = Router();

// Create PostgreSQL pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  },
});

// Middleware to get user from session
// This assumes Better Auth has already authenticated the user
const getUserFromSession = async (req: Request): Promise<string | null> => {
  // Better Auth stores user ID in the session
  // You can access it from the session cookie
  // For now, we'll extract it from a custom header or session
  const userId = req.headers['x-user-id'] as string;
  return userId || null;
};

/**
 * GET /api/personalization/preferences
 * Get user preferences (theme, language, notifications)
 */
router.get('/preferences', async (req: Request, res: Response) => {
  try {
    const userId = await getUserFromSession(req);

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const result = await pool.query(
      'SELECT * FROM "user_preferences" WHERE "userId" = $1',
      [userId]
    );

    if (result.rows.length === 0) {
      // Create default preferences if none exist
      const defaultPrefs = await pool.query(
        `INSERT INTO "user_preferences" ("userId", theme, language)
         VALUES ($1, 'system', 'en')
         RETURNING *`,
        [userId]
      );
      return res.json(defaultPrefs.rows[0]);
    }

    res.json(result.rows[0]);
  } catch (error) {
    console.error('Error fetching preferences:', error);
    res.status(500).json({ error: 'Failed to fetch preferences' });
  }
});

/**
 * PUT /api/personalization/preferences
 * Update user preferences
 */
router.put('/preferences', async (req: Request, res: Response) => {
  try {
    const userId = await getUserFromSession(req);

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const {
      theme,
      language,
      chatHistoryEnabled,
      emailNotifications,
      chatNotifications
    } = req.body;

    const result = await pool.query(
      `INSERT INTO "user_preferences" ("userId", theme, language, "chatHistoryEnabled", "emailNotifications", "chatNotifications")
       VALUES ($1, $2, $3, $4, $5, $6)
       ON CONFLICT ("userId")
       DO UPDATE SET
         theme = COALESCE($2, "user_preferences".theme),
         language = COALESCE($3, "user_preferences".language),
         "chatHistoryEnabled" = COALESCE($4, "user_preferences"."chatHistoryEnabled"),
         "emailNotifications" = COALESCE($5, "user_preferences"."emailNotifications"),
         "chatNotifications" = COALESCE($6, "user_preferences"."chatNotifications"),
         "updatedAt" = CURRENT_TIMESTAMP
       RETURNING *`,
      [userId, theme, language, chatHistoryEnabled, emailNotifications, chatNotifications]
    );

    res.json(result.rows[0]);
  } catch (error) {
    console.error('Error updating preferences:', error);
    res.status(500).json({ error: 'Failed to update preferences' });
  }
});

/**
 * GET /api/personalization/profile
 * Get user profile
 */
router.get('/profile', async (req: Request, res: Response) => {
  try {
    const userId = await getUserFromSession(req);

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    // Get user basic info + extended profile
    const userResult = await pool.query(
      'SELECT id, email, name, image, "emailVerified", "createdAt" FROM "user" WHERE id = $1',
      [userId]
    );

    if (userResult.rows.length === 0) {
      return res.status(404).json({ error: 'User not found' });
    }

    const profileResult = await pool.query(
      'SELECT * FROM "user_profile" WHERE "userId" = $1',
      [userId]
    );

    const skillsResult = await pool.query(
      'SELECT * FROM "user_skills" WHERE "userId" = $1',
      [userId]
    );

    const preferencesResult = await pool.query(
      'SELECT * FROM "user_preferences" WHERE "userId" = $1',
      [userId]
    );

    res.json({
      user: userResult.rows[0],
      profile: profileResult.rows[0] || null,
      skills: skillsResult.rows[0] || null,
      preferences: preferencesResult.rows[0] || null
    });
  } catch (error) {
    console.error('Error fetching profile:', error);
    res.status(500).json({ error: 'Failed to fetch profile' });
  }
});

/**
 * PUT /api/personalization/profile
 * Update user profile
 */
router.put('/profile', async (req: Request, res: Response) => {
  try {
    const userId = await getUserFromSession(req);

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const {
      name,
      displayName,
      bio,
      phoneNumber,
      location,
      website,
      socialLinks
    } = req.body;

    // Update user table (name)
    if (name !== undefined) {
      await pool.query(
        'UPDATE "user" SET name = $1, "updatedAt" = CURRENT_TIMESTAMP WHERE id = $2',
        [name, userId]
      );
    }

    // Update or insert profile
    const profileResult = await pool.query(
      `INSERT INTO "user_profile" ("userId", "displayName", bio, "phoneNumber", location, website, "socialLinks")
       VALUES ($1, $2, $3, $4, $5, $6, $7)
       ON CONFLICT ("userId")
       DO UPDATE SET
         "displayName" = COALESCE($2, "user_profile"."displayName"),
         bio = COALESCE($3, "user_profile".bio),
         "phoneNumber" = COALESCE($4, "user_profile"."phoneNumber"),
         location = COALESCE($5, "user_profile".location),
         website = COALESCE($6, "user_profile".website),
         "socialLinks" = COALESCE($7, "user_profile"."socialLinks"),
         "updatedAt" = CURRENT_TIMESTAMP
       RETURNING *`,
      [userId, displayName, bio, phoneNumber, location, website, socialLinks ? JSON.stringify(socialLinks) : null]
    );

    res.json(profileResult.rows[0]);
  } catch (error) {
    console.error('Error updating profile:', error);
    res.status(500).json({ error: 'Failed to update profile' });
  }
});

/**
 * PUT /api/personalization/skills
 * Update user skills and experience
 */
router.put('/skills', async (req: Request, res: Response) => {
  try {
    const userId = await getUserFromSession(req);

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const {
      softwareExperience,
      hardwareExperience,
      programmingLevel,
      programmingLanguages,
      learningGoals,
      industryBackground
    } = req.body;

    const result = await pool.query(
      `INSERT INTO "user_skills" ("userId", "softwareExperience", "hardwareExperience", "programmingLevel", "programmingLanguages", "learningGoals", "industryBackground")
       VALUES ($1, $2, $3, $4, $5, $6, $7)
       ON CONFLICT ("userId")
       DO UPDATE SET
         "softwareExperience" = COALESCE($2, "user_skills"."softwareExperience"),
         "hardwareExperience" = COALESCE($3, "user_skills"."hardwareExperience"),
         "programmingLevel" = COALESCE($4, "user_skills"."programmingLevel"),
         "programmingLanguages" = COALESCE($5, "user_skills"."programmingLanguages"),
         "learningGoals" = COALESCE($6, "user_skills"."learningGoals"),
         "industryBackground" = COALESCE($7, "user_skills"."industryBackground"),
         "updatedAt" = CURRENT_TIMESTAMP
       RETURNING *`,
      [userId, softwareExperience, hardwareExperience, programmingLevel, programmingLanguages, learningGoals, industryBackground]
    );

    res.json(result.rows[0]);
  } catch (error) {
    console.error('Error updating skills:', error);
    res.status(500).json({ error: 'Failed to update skills' });
  }
});

/**
 * POST /api/personalization/photo
 * Upload profile photo (base64)
 */
router.post('/photo', async (req: Request, res: Response) => {
  try {
    const userId = await getUserFromSession(req);

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const { photoData } = req.body;

    if (!photoData) {
      return res.status(400).json({ error: 'Photo data required' });
    }

    // For now, store base64 image directly in the database
    // In production, upload to a service like Cloudinary or S3
    const result = await pool.query(
      'UPDATE "user" SET image = $1, "updatedAt" = CURRENT_TIMESTAMP WHERE id = $2 RETURNING *',
      [photoData, userId]
    );

    res.json({ image: result.rows[0].image });
  } catch (error) {
    console.error('Error uploading photo:', error);
    res.status(500).json({ error: 'Failed to upload photo' });
  }
});

/**
 * DELETE /api/personalization/photo
 * Remove profile photo
 */
router.delete('/photo', async (req: Request, res: Response) => {
  try {
    const userId = await getUserFromSession(req);

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    await pool.query(
      'UPDATE "user" SET image = NULL, "updatedAt" = CURRENT_TIMESTAMP WHERE id = $2',
      [userId]
    );

    res.json({ success: true });
  } catch (error) {
    console.error('Error removing photo:', error);
    res.status(500).json({ error: 'Failed to remove photo' });
  }
});

export default router;
