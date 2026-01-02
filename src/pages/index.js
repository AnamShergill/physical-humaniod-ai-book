import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import HeroSection from '@site/src/components/Homepage/HeroSection';
import FeaturesSection from '@site/src/components/Homepage/FeaturesSection';
import CTASection from '@site/src/components/Homepage/CTASection';

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An AI-native textbook covering Physical AI & Humanoid Robotics">
      <HeroSection />
      <FeaturesSection />
      <CTASection />
    </Layout>
  );
}
