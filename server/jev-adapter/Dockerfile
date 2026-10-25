FROM node:22-alpine
WORKDIR /app
COPY jev-client.mjs server.mjs ./
ENV NODE_ENV=production PORT=8787
USER node
EXPOSE 8787
CMD ["node", "server.mjs"]
