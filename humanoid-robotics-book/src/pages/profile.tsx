import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../context/AuthContext';
import UserProfile from '../components/Auth/UserProfile';

export default function ProfilePage() {
  return (
    <AuthProvider>
      <Layout title="Profile" description="Manage your account settings">
        <UserProfile />
      </Layout>
    </AuthProvider>
  );
}
